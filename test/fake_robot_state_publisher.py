import rclpy
import time
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import List

class FakeRobotStatePublisher(Node):
    """ROS2 node for publishing fake ROBOT STATES (observations) to test get_observation()."""

    def __init__(self) -> None:
        super().__init__("fake_robot_state_publisher")
        self.PUBLISHING_RATE = 30  # Hz
        
        self.arm_joint_names: List[str] = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", 
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]
        
        # 这些是 FrankaFr3RobotiqGripperConfig 默认订阅的**状态**话题
        self.arm_state_publisher = self.create_publisher(JointState, "/franka/joint_states", 10)
        self.gripper_state_publisher = self.create_publisher(JointState, "/gripper/joint_states", 10)

        self.get_logger().info("Publishing FAKE ROBOT state (observations) for testing.")
        self.timer = self.create_timer(1 / self.PUBLISHING_RATE, self.publish_state)
        self.start_time = time.time()

    def publish_state(self) -> None:
        """Publish fake current joint states (arm + gripper)."""
        t = time.time() - self.start_time
        
        # 1. 伪造机械臂状态 (假设目前都在 0 附近振荡)
        arm_joints = [0.2 * math.cos(t + i) for i in range(7)]
        
        arm_state_msg = JointState()
        arm_state_msg.header.stamp = self.get_clock().now().to_msg()
        arm_state_msg.name = self.arm_joint_names
        arm_state_msg.position = [float(v) for v in arm_joints]
        
        # 2. 伪造夹爪状态 (Robotiq的真实反馈是角度, 所以我们发一个0.0到0.8之间变化的值)
        gripper_pos = 0.8 * ((math.sin(t * 2.0) + 1.0) / 2.0)
        
        gripper_state_msg = JointState()
        gripper_state_msg.header.stamp = self.get_clock().now().to_msg()
        # 夹爪话题的position列表里，配置文件 gripper_state_joint_index 默认读第0个元素
        gripper_state_msg.name = ["robotiq_85_left_knuckle_joint"] 
        gripper_state_msg.position = [float(gripper_pos)]
        
        # 发布模拟状态
        self.arm_state_publisher.publish(arm_state_msg)
        self.gripper_state_publisher.publish(gripper_state_msg)

        # 控制打印频率
        if int(t * self.PUBLISHING_RATE) % 30 == 0:
            self.get_logger().info(f"Fake State -> Arm: {[round(j, 3) for j in arm_joints]}, Gripper: {gripper_pos:.3f}")

def main(args=None):
    rclpy.init(args=args)

    try:
        fake_state_publisher = FakeRobotStatePublisher()
    except Exception as e:
        print(f"Failed to initialize fake state publisher: {e}")
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(fake_state_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        fake_state_publisher.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
