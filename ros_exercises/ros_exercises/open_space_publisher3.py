# 导入所需的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace

class OpenSpacePublisherNode(Node):
    """
    一个节点，订阅 LaserScan 话题，找到最远的点，并发布 OpenSpace 消息。
    这个版本使用ROS参数来配置话题名称。
    """
    def __init__(self):
        super().__init__('open_space_publisher')

        # 声明并获取参数
        self.declare_parameter('subscriber_topic', 'fake_scan')
        self.declare_parameter('publisher_topic', 'open_space')
        subscriber_topic_name = self.get_parameter('subscriber_topic').get_parameter_value().string_value
        publisher_topic_name = self.get_parameter('publisher_topic').get_parameter_value().string_value

        # 使用参数创建订阅者和发布者
        self.subscription = self.create_subscription(
            LaserScan,
            subscriber_topic_name,
            self.scan_callback,
            10)
        self.open_space_publisher = self.create_publisher(
            OpenSpace,
            publisher_topic_name,
            10)

        self.get_logger().info(f"Listening to '{subscriber_topic_name}' and publishing to '{publisher_topic_name}'.")

    def scan_callback(self, scan_msg):
        # 这部分处理逻辑完全不变
        max_range = max(scan_msg.ranges)
        max_range_index = scan_msg.ranges.index(max_range)
        corresponding_angle = scan_msg.angle_min + (max_range_index * scan_msg.angle_increment)

        open_space_msg = OpenSpace()
        open_space_msg.angle = corresponding_angle
        open_space_msg.distance = max_range

        self.open_space_publisher.publish(open_space_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
