import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FlangePoseReader(Node):
    def __init__(self):
        super().__init__('flange_pose_reader')

        # 1. 创建内部 Buffer 来存储 TF 树的过去几秒的状态
        self.tf_buffer = Buffer()
        
        # 2. 创建监听器，它会在后台自动订阅 /tf 和 /tf_static 并填充到 buffer 中
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 创建定时器，按照 30Hz 查询并读取一次，与机械臂状态发布频率对齐
        self.timer = self.create_timer(1.0 / 30.0, self.get_pose)

    def get_pose(self):
        try:
            # 核心查询：查询目标坐标系(fr3_link8) 在 参考坐标系(fr3_link0) 下的位姿
            t = self.tf_buffer.lookup_transform(
                'fr3_link0',   # 参考系 (也就是基座)
                'fr3_link8',   # 目标系 (也就是法兰盘)
                rclpy.time.Time() # rclpy.time.Time() 表示要获取最新可用的数据
            )

            # --- 提取位置 (Translation) ---
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            # --- 提取姿态 (Rotation - 四元数) ---
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # 打印出来
            self.get_logger().info(f"法兰位置 XYZ: [{x:.4f}, {y:.4f}, {z:.4f}]")
            self.get_logger().info(f"法兰姿态 四元数: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")

        except TransformException as ex:
            # 刚启动时如果还没收到 TF 数据，会报这个错，属于正常现象
            self.get_logger().warn(f'还未拿到位齐 TF 数据: {ex}')

def main():
    rclpy.init()
    node = FlangePoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()