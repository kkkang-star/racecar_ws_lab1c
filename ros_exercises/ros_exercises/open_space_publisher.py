# 导入所需的库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # 导入 LaserScan 用于订阅
from std_msgs.msg import Float32     # 导入 Float32 用于发布

class OpenSpacePublisherNode(Node):
    """
    一个节点，订阅 /fake_scan 话题，找到最远的扫描点，
    并将其距离和角度分别发布到 /open_space/distance 和 /open_space/angle 话题。
    """
    def __init__(self):
        # 初始化父类，并设置节点名称
        super().__init__('open_space_publisher')
        
        # 创建一个订阅者，监听 /fake_scan 话题
        self.subscription = self.create_subscription(
            LaserScan,
            'fake_scan',
            self.scan_callback,  # 收到消息时调用 scan_callback 函数
            10)
        
        # 创建两个发布者
        self.distance_publisher = self.create_publisher(Float32, 'open_space/distance', 10)
        self.angle_publisher = self.create_publisher(Float32, 'open_space/angle', 10)

    def scan_callback(self, scan_msg):
        """
        处理 LaserScan 消息的回调函数。
        """
        # 1. 找到最远的距离值
        # scan_msg.ranges 是一个包含所有距离读数的列表
        # 使用 Python 内置的 max() 函数可以轻松找到最大值
        max_range = max(scan_msg.ranges)
        
        # 2. 找到最大值对应的索引（位置）
        # 使用列表的 .index() 方法找到最大值在列表中的位置
        max_range_index = scan_msg.ranges.index(max_range)
        
        # 3. 根据索引计算对应的角度
        # 角度的计算公式为：起始角度 + 索引 * 角度增量
        corresponding_angle = scan_msg.angle_min + (max_range_index * scan_msg.angle_increment)
        
        # 4. 准备要发布的新消息
        distance_msg = Float32()
        distance_msg.data = max_range
        
        angle_msg = Float32()
        angle_msg.data = corresponding_angle
        
        # 5. 发布两个消息
        self.distance_publisher.publish(distance_msg)
        self.angle_publisher.publish(angle_msg)
        
        # 打印日志，方便调试
        self.get_logger().info(
            f'Longest range: {max_range:.2f} at angle: {corresponding_angle:.2f} rad'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OpenSpacePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
