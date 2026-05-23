import rclpy
import time
import math
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from typing import List, Tuple

class FakeGelloPublisher(Node):
    """ROS2 node for publishing fake inference outputs to test the pipeline."""

    def __init__(self) -> None:
        super().__init__("fake_gello_publisher")
        self.PUBLISHING_RATE = 30  # Hz
        self.joint_names: List[str] = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self.arm_joint_publisher = self.create_publisher(JointState, "gello/joint_states", 10)
        self.gripper_joint_publisher = self.create_publisher(
            Float32, "gripper/gripper_client/target_gripper_width_percent", 10
        )
        self.gripper_raw_publisher = self.create_publisher(Float32, "gello/gripper_position", 10)

        self.get_logger().info("Publishing FAKE GELLO joint states for testing.")
        self.timer = self.create_timer(1 / self.PUBLISHING_RATE, self.publish_joint_jog)
        self.start_time = time.time()

    def infer_joint_and_gripper_positions(self) -> Tuple[List[float], float]:
        """Generate fake model inference data."""
        t = time.time() - self.start_time
        
        # 关节角度小范围规律变化，振幅为 0.1 rad
        joints = [0.1 * math.sin(t + i) for i in range(7)]
        
        # Robotiq raw gripper target: 0.0=open, 0.085=max closed.
        # math.sin 范围是 [-1, 1], (sin + 1)/2 范围是 [0, 1]
        gripper = 0.085 * ((math.sin(t * 1.5) + 1.0) / 2.0)
        
        return joints, gripper

    def publish_joint_jog(self) -> None:
        """Publish current joint states and gripper position."""
        gello_arm_joints, gripper_position = self.infer_joint_and_gripper_positions()

        # 发布机械臂关节状态
        arm_joint_states = JointState()
        arm_joint_states.header.stamp = self.get_clock().now().to_msg()
        arm_joint_states.name = self.joint_names
        arm_joint_states.header.frame_id = "fr3_link0"
        arm_joint_states.position = [float(v) for v in gello_arm_joints]

        # Publish gripper control as open-width percent, and raw position for recording.
        gripper_position = float(min(0.085, max(0.0, gripper_position)))
        gripper_joint_states = Float32()
        gripper_joint_states.data = 1.0 - (gripper_position / 0.085)
        gripper_raw_states = Float32()
        gripper_raw_states.data = gripper_position
        
        self.arm_joint_publisher.publish(arm_joint_states)
        self.gripper_joint_publisher.publish(gripper_joint_states)
        self.gripper_raw_publisher.publish(gripper_raw_states)
        
        # 防止刷屏，可以一秒钟打印一次状态 (约30帧)
        if int(t := (time.time() - self.start_time) * self.PUBLISHING_RATE) % 30 == 0:
            self.get_logger().info(
                "Fake output -> Arm: %s, Gripper target raw: %.3f, open percent: %.3f",
                [round(j, 3) for j in gello_arm_joints],
                gripper_raw_states.data,
                gripper_joint_states.data,
            )

def main(args=None):
    rclpy.init(args=args)

    try:
        fake_publisher = FakeGelloPublisher()
    except Exception as e:
        print(f"Failed to initialize fake publisher: {e}")
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(fake_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        fake_publisher.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()


# python -m tmp.test_inference
