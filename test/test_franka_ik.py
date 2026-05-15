import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK

class FrankaIKSolver(Node):
    def __init__(self):
        super().__init__('franka_ik_solver')
        
        # 创建一个客户端，连接到 MoveIt 自动提供的官方 IK 服务
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 等待服务上线（前提是你的终端里运行了 moveit.launch.py 相关的节点）
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt 的 /compute_ik 服务上线...')

    def solve_ik(self, target_xyz, target_quat):
        # 1. 构造官方要求的请求包
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'fr3_arm'      # Franka 在 MoveIt 里的标准规划组叫这个
        req.ik_request.ik_link_name = 'fr3_link8'  # 目标对齐的连杆（法兰面）
        
        # 2. 填入你想要到达的目标位姿
        pose = PoseStamped()
        pose.header.frame_id = 'fr3_link0'         # 基于基座的坐标
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = target_xyz[0]
        pose.pose.position.y = target_xyz[1]
        pose.pose.position.z = target_xyz[2]
        
        pose.pose.orientation.x = target_quat[0]
        pose.pose.orientation.y = target_quat[1]
        pose.pose.orientation.z = target_quat[2]
        pose.pose.orientation.w = target_quat[3]
        
        req.ik_request.pose_stamped = pose
        
        # 3. 设置求解超时时间 (官方配的是 0.005 秒，这里我们给稍微宽裕点)
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 50000000  # 50 ms

        self.get_logger().info('正在呼叫官方求解器...')
        
        # 4. 发送请求并阻塞等待结果
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        
        # 5. 解析官方返回的数据
        if response.error_code.val == response.error_code.SUCCESS:
            # 拿到 7 个机械臂电机的角度
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position
            
            self.get_logger().info('求解成功！')
            for name, pos in zip(joint_names, joint_positions):
                print(f"  {name}: {pos:.4f} rad")
            return joint_positions
        else:
            self.get_logger().error(f'求解失败！错误码: {response.error_code.val}')
            return None

def main():
    rclpy.init()
    node = FrankaIKSolver()
    
    # 测试一下你刚才通过 TF 读取到的那个法兰真实坐标！
    # (如果用它真实的当前坐标去求 IK，算出来的结果应该也就是机器人的当前关节角度)
    xyz = [0.3069, 0.0000, 0.5903]
    quat = [0.924, -0.383, 0.000, 0.000]
    
    node.solve_ik(xyz, quat)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()