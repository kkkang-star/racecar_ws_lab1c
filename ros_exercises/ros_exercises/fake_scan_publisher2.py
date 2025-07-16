# 导入所需的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import random
import math

class FakeScanPublisherNode(Node):
    """
    一个节点，发布伪造的LaserScan数据。
    这个版本使用了ROS参数来配置其行为，包括话题名称。
    """
    def __init__(self):
        super().__init__('fake_scan_publisher')

        # --- 1. 声明所有可配置的参数，并提供默认值 ---
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('scan_topic', 'fake_scan')          # 激光雷达话题名
        self.declare_parameter('range_topic', 'range_test')        # 数组长度话题名
        self.declare_parameter('angle_min', (-2/3) * math.pi)
        self.declare_parameter('angle_max', (2/3) * math.pi)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_increment', (1/300) * math.pi)

        # --- 2. 获取参数的当前值 ---
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        scan_topic_name = self.get_parameter('scan_topic').get_parameter_value().string_value
        range_topic_name = self.get_parameter('range_topic').get_parameter_value().string_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value

        # --- 3. 使用获取到的参数 ---
        # 创建两个发布者，使用参数化的话题名称
        self.scan_publisher = self.create_publisher(LaserScan, scan_topic_name, 10)
        self.range_publisher = self.create_publisher(Float32, range_topic_name, 10)

        # LaserScan 的其他静态参数
        self.time_increment = 0.0
        self.scan_time = 1.0 / publish_rate # 使用参数来计算 scan_time

        # 计算 ranges 数组的长度
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # 使用参数来创建定时器
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Fake Scan Publisher started. Publishing to '{scan_topic_name}' at {publish_rate} Hz.")


    def timer_callback(self):
        # 构建并发布 LaserScan 消息 (这部分逻辑不变)
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = self.time_increment
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = [random.uniform(self.range_min, self.range_max) for _ in range(self.num_ranges)]
        self.scan_publisher.publish(scan_msg)

        # 构建并发布 Float32 消息 (这部分逻辑不变)
        range_msg = Float32()
        range_msg.data = float(len(scan_msg.ranges))
        self.range_publisher.publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()