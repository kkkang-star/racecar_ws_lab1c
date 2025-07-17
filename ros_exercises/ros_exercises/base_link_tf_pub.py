import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import tf_transformations
# 导入我们之前写的两个工具函数
from .dynamic_tf_cam_publisher import transform_to_matrix, matrix_to_transform

class BaseLinkTFPub(Node):
    def __init__(self):
        super().__init__('base_link_tf_pub')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 预计算 left_cam -> base_link 的变换矩阵
        # 这是 base_link -> left_cam 的逆
        T_base_to_left = tf_transformations.translation_matrix([0.0, 0.05, 0.0])
        self.T_left_to_base = np.linalg.inv(T_base_to_left)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # 1. 监听 odom -> left_cam 的变换
            t_odom_to_left = self.tf_buffer.lookup_transform(
                'odom', 'left_cam', rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform odom to left_cam: {ex}')
            return
            
        # 2. 转换为矩阵
        T_odom_to_left = transform_to_matrix(t_odom_to_left)
        
        # 3. 计算 odom -> base_link_2 的变换
        # T_odom_base2 = T_odom_left * T_left_base
        T_odom_to_base2 = np.dot(T_odom_to_left, self.T_left_to_base)
        
        # 4. 转换回消息并发布
        t_odom_to_base2 = matrix_to_transform(
            T_odom_to_base2,
            self.get_clock().now().to_msg(),
            'odom',
            'base_link_2'
        )
        self.tf_broadcaster.sendTransform(t_odom_to_base2)

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkTFPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
