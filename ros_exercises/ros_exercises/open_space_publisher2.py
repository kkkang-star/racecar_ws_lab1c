# 导入所需的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# ***** 修改导入语句 *****
from custom_msgs.msg import OpenSpace  # 导入我们自己的自定义消息

class OpenSpacePublisherNode(Node):
    """
    订阅 /fake_scan 话题，找到最远的扫描点，
    并将其距离和角度封装在一个 OpenSpace 消息中，发布到 /open_space 话题。
    """
    def __init__(self):
        super().__init__('open_space_publisher')

        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.scan_callback,
            10)

        # ***** 修改发布者 *****
        # 移除两个旧的发布者，创建一个新的
        self.open_space_publisher = self.create_publisher(OpenSpace, 'open_space', 10)

    def scan_callback(self, scan_msg):
        max_range = max(scan_msg.ranges)
        max_range_index = scan_msg.ranges.index(max_range)
        corresponding_angle = scan_msg.angle_min + (max_range_index * scan_msg.angle_increment)

        # ***** 修改消息创建和发布逻辑 *****
        # 1. 创建 OpenSpace 消息实例
        open_space_msg = OpenSpace()

        # 2. 填充数据 (注意题目要求的顺序)
        open_space_msg.angle = corresponding_angle
        open_space_msg.distance = max_range

        # 3. 发布单个自定义消息
        self.open_space_publisher.publish(open_space_msg)

        # 4. 更新日志
        self.get_logger().info(
            f'Publishing to /open_space: angle={open_space_msg.angle:.2f}, distance={open_space_msg.distance:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
