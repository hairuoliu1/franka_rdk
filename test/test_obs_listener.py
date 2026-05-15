import sys
from pathlib import Path
# 把项目的根目录临时加到环境变量里，方便读取 src 下的包
sys.path.append(str(Path(__file__).resolve().parent.parent))

import time
import rclpy
from src.robots.franka_fr3_robotiq_gripper.franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripper
from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

def test_observation():
    print("初始化 rclpy...")
    # rclpy 在类内部自己也会 init，但提前 init 或者保持上下文是个好习惯
    if not rclpy.ok():
        rclpy.init()

    print("创建 FrankaFr3RobotiqGripper 实例并连接...")
    # 这里可以使用默认配置，它会默认订阅真机（或仿真）对应的 arm 和 gripper state 话题
    config = FrankaFr3RobotiqGripperConfig()
    
    # 实例化机器人对象
    robot = FrankaFr3RobotiqGripper(config)
    
    # 连接到 ROS 话题
    robot.connect()

    print(f"robot.is_connected: {robot.is_connected}")
    print("开始获取观测数据 get_observation() (测试 100 次以 10Hz 频率打印)：\n")

    try:
        for i in range(100):
            # 获取观测数据
            obs = robot.get_observation()
            
            # obs 的 key 是 "joint_positions_0" 到 "joint_positions_7" 共 8 个自由度
            # 前 7 个是机械臂关节，第 8 个是夹爪状态
            arm_joints = [obs[f"joint_positions_{j}"] for j in range(7)]
            gripper_state = obs.get("joint_positions_7", 0.0)

            print(f"[Obs Iter {i:03d}] Arm: {[round(v, 3) for v in arm_joints]}, Gripper: {gripper_state:.3f}")
            
            time.sleep(0.1) # 10Hz 打印

    except KeyboardInterrupt:
        print("\n用户中断观测测试...")
    finally:
        print("断开连接并清理 rclpy...")
        robot.disconnect()
        if rclpy.ok():
            rclpy.shutdown()
        print("测试完成。")

if __name__ == "__main__":
    test_observation()
