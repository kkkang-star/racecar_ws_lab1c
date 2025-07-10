import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # 我们收发的消息类型都是Float32
import math  # 导入math库用于计算自然对数

class SimpleSubscriberNode(Node):
    """
    订阅 'my_random_float' 话题，计算接收到的数据的自然对数，
    然后将结果发布到 'random_float_log' 话题。
    """
    def __init__(self):
        # 初始化父类，并设置节点名称为 'simple_subscriber'
        super().__init__('simple_subscriber')

        # 创建一个订阅者
        # 它订阅 Float32 类型的消息，话题名为 'my_random_float'
        # 每当收到消息，就会调用 self.listener_callback 函数
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)

        # 创建一个发布者
        # 它发布 Float32 类型的消息，话题名为 'random_float_log'
        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)

    def listener_callback(self, msg):
        """
        订阅者的回调函数，在此处处理接收到的消息并发布新消息。
        """
        # 在终端打印日志，显示接收到的数据
        self.get_logger().info(f'Received: "{msg.data}"')

        # 准备一个新的消息用于发布
        log_msg = Float32()

        # 计算接收数据的自然对数
        # 注意：log(x) 要求 x > 0，所以我们加一个检查以避免数学错误
        if msg.data > 0:
            log_msg.data = math.log(msg.data)

            # 发布计算结果
            self.publisher_.publish(log_msg)

            # 在终端打印日志，显示正在发布的新数据
            self.get_logger().info(f'Publishing log: "{log_msg.data}"')
        else:
            # 如果接收到的数据小于等于0，则打印警告，不进行计算和发布
            self.get_logger().warn(f'Received non-positive value ({msg.data}), skipping log calculation.')

def main(args=None):
    # 初始化rclpy库
    rclpy.init(args=args)

    # 创建节点实例
    simple_subscriber = SimpleSubscriberNode()

    # 进入循环，等待回调函数被调用
    rclpy.spin(simple_subscriber)

    # 清理并关闭
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
