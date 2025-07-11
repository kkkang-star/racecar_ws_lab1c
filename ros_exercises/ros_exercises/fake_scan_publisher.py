# 导入所需的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # 导入 LaserScan 消息类型
from std_msgs.msg import Float32     # 导入 Float32 消息类型
import random
import math

class FakeScanPublisherNode(Node):
    """
    一个节点，发布伪造的LaserScan数据和一个包含ranges数组长度的Float32数据。
    """
    def __init__(self):
        # 初始化父类，并设置节点名称
        super().__init__('fake_scan_publisher')
        
        # 创建两个发布者
        self.scan_publisher = self.create_publisher(LaserScan, 'fake_scan', 10)
        self.range_publisher = self.create_publisher(Float32, 'range_test', 10)
        
        # --- LaserScan 消息的静态参数 ---
        self.angle_min = (-2/3) * math.pi  # -120 degrees
        self.angle_max = (2/3) * math.pi   # +120 degrees
        self.angle_increment = (1/300) * math.pi # 角度增量
        self.time_increment = 0.0  # 测量之间的时间，可以为0
        self.scan_time = 0.05      # 20hz, 每次扫描之间的时间
        self.range_min = 1.0       # 最小测量距离
        self.range_max = 10.0      # 最大测量距离

        # --- 计算 ranges 数组的长度 ---
        # 关键点: 为了包含 angle_min 和 angle_max 两个端点，需要 +1
        # 这解决了题目中提到的 "off-by-1" (差一) 错误
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # 创建一个20Hz的定时器
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # --- 1. 构建并发布 LaserScan 消息 ---
        scan_msg = LaserScan()
        
        # 填充 header
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        
        # 填充 LaserScan 的几何参数
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # 生成随机的 ranges 数据
        # 使用列表推导式快速生成一个包含随机浮点数的列表
        scan_msg.ranges = [random.uniform(self.range_min, self.range_max) for _ in range(self.num_ranges)]
        
        # 发布 LaserScan 消息
        self.scan_publisher.publish(scan_msg)
        
        # --- 2. 构建并发布 Float32 消息 ---
        range_msg = Float32()
        
        # 注意: 题目要求将长度转换为 float 类型
        range_msg.data = float(len(scan_msg.ranges))
        
        # 发布 Float32 消息
        self.range_publisher.publish(range_msg)
        
        # 打印日志
        self.get_logger().info(f"Publishing fake scan with {len(scan_msg.ranges)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
