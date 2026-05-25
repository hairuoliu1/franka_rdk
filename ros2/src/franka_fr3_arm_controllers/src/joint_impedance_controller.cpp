// Copyright (c) 2025 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_fr3_arm_controllers/joint_impedance_controller.hpp>

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

using std::placeholders::_1;

namespace franka_fr3_arm_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(namespace_prefix_ + arm_id_ + "_joint" + std::to_string(i) +
                           "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates_();
  Vector7d q_goal;
  Vector7d tau_d_calculated;

  // 默认位置保持，直到接收到有效的关节状态并初始化运动生成器
  if (!motion_generator_initialized_) {
    // After starting the controller we wait for valid joint states from the input topic
    // Until we get valid joint states we will hold the current position
    motion_generator_initialized_ = initializeMotionGenerator_();
    if (!motion_generator_initialized_) {
      if (!hold_position_logged_) {
        q_hold_ = q_;
        hold_position_logged_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Position hold active until GELLO connects.");
      }
      tau_d_calculated = calculateTauDGains_(q_hold_);
      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau_d_calculated(i));
      }
      return controller_interface::return_type::OK;
    }
    hold_position_logged_ = false;
  }

  if (!move_to_start_position_finished_) {
    // We have received valid joint states and initialized the motion generator
    // Now we move smoothly to the first joint position received from the input topic
    auto trajectory_time = this->get_node()->now() - start_time_;
    auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
    move_to_start_position_finished_ = motion_generator_output.second;

    q_goal = motion_generator_output.first;
  }

  if (move_to_start_position_finished_) {
    // After reaching the start position we follow the joint position from the input topic
    // This is the normal operation mode of the controller
    // 取消了自动shutdown，改为当gello位置值无效时发送警告并继续使用最后一个有效的gello位置值
    if (!gello_position_values_valid_) {
      if (!gello_timeout_logged_) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "Gello timeout: keeping last valid position after %.1fs gap.",
                    (this->get_node()->now() - last_joint_state_time_).seconds());
        gello_timeout_logged_ = true;
      }
    } else {
      gello_timeout_logged_ = false;
    }
    for (int i = 0; i < num_joints; ++i) {
      q_goal(i) = gello_position_values_[i];
    }
  }

  tau_d_calculated = calculateTauDGains_(q_goal);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

void JointImpedanceController::jointStateCallback_(const sensor_msgs::msg::JointState msg) {
  if (last_joint_state_time_.seconds() == 0.0) {
    return;
  }

  if (msg.position.size() < gello_position_values_.size()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received joint state size is smaller than expected size.");
    return;
  }

  std::copy(msg.position.begin(), msg.position.begin() + gello_position_values_.size(),
            gello_position_values_.begin());

  validateGelloPositions_(msg);
  last_joint_state_time_ = msg.header.stamp;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::string>("gello_joint_states_topic", "gello/joint_states");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  namespace_prefix_ = get_node()->get_namespace();
  if (namespace_prefix_ == "/" || namespace_prefix_.empty()) {
    namespace_prefix_.clear();
  } else {
    // Remove leading slash and add trailing underscore
    namespace_prefix_ = namespace_prefix_.substr(1) + "_";
  }

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto k_alpha = get_node()->get_parameter("k_alpha").as_double();
  gello_joint_states_topic_ = get_node()->get_parameter("gello_joint_states_topic").as_string();

  if (!validateGains_(k_gains, "k_gains") || !validateGains_(d_gains, "d_gains")) {
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  if (k_alpha < 0.0 || k_alpha > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_alpha should be in the range [0, 1]");
    return CallbackReturn::FAILURE;
  }

  k_alpha_ = k_alpha;

  dq_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  const auto resolved_gello_topic =
      get_node()->get_node_topics_interface()->resolve_topic_name(gello_joint_states_topic_);
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to GELLO joint states on '%s'.",
              resolved_gello_topic.c_str());
  joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      gello_joint_states_topic_, 1,
      [this](const sensor_msgs::msg::JointState& msg) { jointStateCallback_(msg); });

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  last_joint_state_time_ = get_node()->now();
  dq_filtered_.setZero();
  start_time_ = this->get_node()->now();

  return CallbackReturn::SUCCESS;
}

auto JointImpedanceController::calculateTauDGains_(const Vector7d& q_goal) -> Vector7d {
  dq_filtered_ = (1 - k_alpha_) * dq_filtered_ + k_alpha_ * dq_;
  Vector7d tau_d_calculated;
  tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);

  return tau_d_calculated;
}

bool JointImpedanceController::validateGains_(const std::vector<double>& gains,
                                              const std::string& gains_name) {
  if (gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s parameter not set", gains_name.c_str());
    return false;
  }

  if (gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s should be of size %d but is of size %ld",
                 gains_name.c_str(), num_joints, gains.size());
    return false;
  }

  return true;
}

void JointImpedanceController::validateGelloPositions_(const sensor_msgs::msg::JointState& msg) {
  const double max_time_diff = 1.5; //gello判定阈值默认0.5秒，超过这个时间差就认为gello位置值无效
  auto current_time = get_node()->now();
  auto time_since_last_joint_state = (current_time - last_joint_state_time_).seconds();
  auto time_since_msg_stamp = (current_time - msg.header.stamp).seconds();
  gello_position_values_valid_ =
      (time_since_last_joint_state < max_time_diff && time_since_msg_stamp < max_time_diff);
  if (!gello_position_values_valid_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Gello position values are not valid. Time since last joint state: %f // Time "
                "since message stamp: %f",
                time_since_last_joint_state, time_since_msg_stamp);
  }
}

void JointImpedanceController::updateJointStates_() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool JointImpedanceController::initializeMotionGenerator_() {
  if (!gello_position_values_valid_) {
    // Only send a warning once every 10 seconds in order not to spam the log
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 10 * 1000,
                         "Waiting for valid joint states...");
    return false;
  }

  Vector7d q_goal;
  updateJointStates_();
  for (int i = 0; i < num_joints; ++i) {
    q_goal(i) = gello_position_values_[i];
  }
  RCLCPP_INFO(get_node()->get_logger(), "q_goal of motion generator: [%f, %f, %f, %f, %f, %f, %f]",
              q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5), q_goal(6));

  const double motion_generator_speed_factor = 0.2;
  motion_generator_ = std::make_unique<MotionGenerator>(motion_generator_speed_factor, q_, q_goal);
  return true;
}

}  // namespace franka_fr3_arm_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_fr3_arm_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
