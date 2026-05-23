import rclpy
from rclpy.node import Node
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Float32

DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper_client/target_gripper_width_percent"
DEFAULT_MOVE_ACTION_TOPIC = "robotiq_gripper_controller/gripper_cmd"
# GripperActionController commands robotiq_85_left_knuckle_joint in radians.
ROBOTIQ_2F85_MAX_CLOSED_JOINT_POSITION = 0.8
COMMAND_DEADBAND = 0.002
MIN_OPEN_WIDTH_PERCENT = 0.0
MAX_OPEN_WIDTH_PERCENT = 1.0


class RobotiqGripperClient(Node):
    def __init__(self):
        super().__init__("robotiq_gripper_client")
        self.get_logger().info("Starting Robotiq Gripper Client")
        self.gripper_state_sub = self.create_subscription(
            Float32,
            DEFAULT_GRIPPER_COMMAND_TOPIC,
            self.gripper_state_callback,
            10,
        )
        self.action_client = ActionClient(self, GripperCommand, DEFAULT_MOVE_ACTION_TOPIC)
        self.action_client.wait_for_server()
        self.last_position = -1.0
        self.get_logger().info("Gripper action server is up and running")

    def gripper_state_callback(self, msg):
        open_width_percent = max(
            MIN_OPEN_WIDTH_PERCENT,
            min(MAX_OPEN_WIDTH_PERCENT, float(msg.data)),
        )
        gripper_target_position = ROBOTIQ_2F85_MAX_CLOSED_JOINT_POSITION * (
            1.0 - open_width_percent
        )
        if abs(gripper_target_position - self.last_position) < COMMAND_DEADBAND:
            return
        self.send_gripper_command(gripper_target_position)

    def send_gripper_command(self, gripper_position):
        gripper_position = max(
            0.0, min(ROBOTIQ_2F85_MAX_CLOSED_JOINT_POSITION, float(gripper_position))
        )
        self.last_position = gripper_position
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = gripper_position
        goal_msg.command.max_effort = 1.0
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"Goal rejected with status: {goal_handle.status}")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result))


def main(args=None):
    rclpy.init(args=args)
    gripper_client = RobotiqGripperClient()
    rclpy.spin(gripper_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
