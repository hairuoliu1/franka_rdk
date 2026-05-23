import sys
from pathlib import Path
# 把项目的根目录临时加到环境变量里，方便读取 src 下的包
sys.path.append(str(Path(__file__).resolve().parent.parent))

import time
from src.robots.franka_fr3_robotiq_gripper.franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripper
from src.robots.franka_fr3_robotiq_gripper.config_franka_fr3_robotiq_gripper import FrankaFr3RobotiqGripperConfig

def main():
    print("Initializing Franka FR3 + Robotiq Gripper Test...")
    
    # 禁用相机进行纯机械臂和夹爪状态测试
    config = FrankaFr3RobotiqGripperConfig(cameras={})
    
    robot = FrankaFr3RobotiqGripper(config)
    print("Connecting to robot...")
    robot.connect()
    
    time.sleep(1.0)
    
    try:
        # 获取当前观测
        obs = robot.get_observation()
        print("\n--- Initial Observation ---")
        for i in range(8):
            print(f"Joint {i}: {obs[f'joint_positions_{i}']:.4f}")
            
        print("\nReady to test gripper. The arm will maintain its current position.")
        
        # Robotiq raw 夹爪约定：0.0=张开，0.085=最大闭合。
        for step in ["OPEN (0.0)", "CLOSE (0.085)", "OPEN (0.0)", "CLOSE (0.085)"]:
            target_value = 0.085 if "0.085" in step else 0.0
            input(f"\nPress Enter to send gripper command: {step} ...")
            
            # 读取当前状态，只改变夹爪目标值
            obs = robot.get_observation()
            action = {}
            for i in range(7):
                # 如果没有订阅到机械臂状态，提供0兜底
                action[f"joint_positions_{i}"] = obs.get(f"joint_positions_{i}", 0.0)
            
            # 设置新夹爪目标
            action["joint_positions_7"] = target_value
            
            print(f"Sending action -> Gripper target: {target_value}")
            sent_action = robot.send_action(action)
            
            print("Waiting for movement to complete...")
            # 循环监测状态反馈
            for _ in range(20):
                time.sleep(0.1)
                new_obs = robot.get_observation()
                current_gripper = new_obs["joint_positions_7"]
                print(f"Current gripper feedback: {current_gripper:.4f}", end="\r")
            
            print() # 换行

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        print("\nDisconnecting...")
        robot.disconnect()
        print("Done.")

if __name__ == "__main__":
    main()
    # 记得启动夹爪的 ROS 话题发布器（真机或仿真）来测试这个脚本
    # ros2 launch franka_gripper_manager robotiq_gripper_controller_client.launch.py \
    #  config_file:=/workspace/src/config/robotiq_gripper_config.yaml
