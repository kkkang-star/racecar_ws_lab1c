# 导入所需的库
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # 导入Float32消息类型
import random  # 导入random库用于生成随机数

class SimplePublisherNode(Node):
    """
    一个简单的ROS节点，以20Hz的频率发布一个0.0到10.0之间的随机浮点数。
    """
    def __init__(self):
        # 调用父类的构造函数，并设置节点名称为 'simple_publisher'
        super().__init__('simple_publisher')

        # 创建一个发布者
        # 参数: 消息类型(Float32), 主题名称('my_random_float'), QoS队列大小(10)
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', 10)

        # 设置发布频率为20Hz (1/20 = 0.05秒)
        timer_period = 0.05

        # 创建一个定时器，每隔 timer_period 秒调用一次 timer_callback 函数
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        定时器回调函数，在此处生成并发布消息。
        """
        # 创建一个Float32类型的消息对象
        msg = Float32()

        # 生成一个0.0到10.0之间的随机浮点数，并赋值给消息的data字段
        msg.data = random.uniform(0.0, 10.0)

        # 发布消息
        self.publisher_.publish(msg)

        # 在终端打印日志，方便调试，显示正在发布的数据
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    # 初始化rclpy库
    rclpy.init(args=args)

    # 创建节点实例
    simple_publisher = SimplePublisherNode()

    # 进入循环，等待回调函数被调用
    rclpy.spin(simple_publisher)

    # 节点关闭时，销毁节点并关闭rclpy
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
