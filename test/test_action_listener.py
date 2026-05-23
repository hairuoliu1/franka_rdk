"""
这是一个简单的测试脚本，用于验证 GelloROSLeader 是否能够正确监听 ROS 话题并记录动作数据。
测试action的格式是否正确，并且能够以10Hz的频率打印出来。
配合test_inference.py一起使用，可以验证从监听到的动作数据能够被正确地传递到模型推理中。
"""
import sys
from pathlib import Path
# 把项目的根目录临时加到环境变量里，方便读取 src 下的包
sys.path.append(str(Path(__file__).resolve().parent.parent))

import time
import rclpy
from src.teleoperators.gello_leader.gello_ros_leader import GelloRosLeader
from src.teleoperators.gello_leader.config_gello_ros_leader import GelloRosLeaderConfig

def test_listener():
    print("初始化 rclpy...")
    rclpy.init()

    print("创建 GelloRosLeader 实例并连接...")
    # 使用默认配置，监听 GELLO arm 和 Robotiq raw gripper target 话题。
    teleop = GelloRosLeader(GelloRosLeaderConfig())
    teleop.connect()

    print(f"teleop.is_connected: {teleop.is_connected}")
    print(f"teleop.is_passive: {teleop.is_passive}")
    print("开始监听伪造的动作数据 (测试 100 次以 10Hz 频率打印)：\n")

    try:
        for i in range(100):
            # 获取记录下来的 action
            action = teleop.get_action()
            
            # 为了更好的输出排版，我们提取手臂的前7个关节和第8个夹爪状态
            arm_joints = [action[f"joint_positions_{j}"] for j in range(7)]
            gripper_state = action.get("joint_positions_7", 0.0)

            print(f"[Iter {i:03d}] Arm: {[round(v, 3) for v in arm_joints]}, Gripper: {gripper_state:.3f}")
            
            time.sleep(0.1) # 10Hz 打印

    except KeyboardInterrupt:
        print("\n用户中断测试...")
    finally:
        print("断开连接并清理 rclpy...")
        teleop.disconnect()
        rclpy.shutdown()
        print("测试完成。")

if __name__ == "__main__":
    test_listener()

# python -m tmp.test_gello_listener
