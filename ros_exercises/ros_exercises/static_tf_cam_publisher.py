import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticTFCamPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_cam_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # --- 变换 1: base_link -> left_cam ---
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'left_cam'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.05 # 左移0.05米
        t1.transform.translation.z = 0.0
        # 旋转为单位四元数 (没有旋转)
        q1 = tf_transformations.quaternion_from_euler(0, 0, 0)
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]

        # --- 变换 2: left_cam -> right_cam ---
        # 从left_cam的视角看，right_cam在y轴负方向0.1米处
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'left_cam'
        t2.child_frame_id = 'right_cam'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = -0.1 # y轴移动-0.1米
        t2.transform.translation.z = 0.0
        # 旋转也为单位四元数
        q2 = tf_transformations.quaternion_from_euler(0, 0, 0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]

        # 一次性广播所有静态变换
        self.tf_static_broadcaster.sendTransform([t1, t2])

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFCamPublisher()
    # 注意: 静态发布者不需要spin，因为它只做一次性工作
    # 但为了让节点保持存活，我们通常还是会spin
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
