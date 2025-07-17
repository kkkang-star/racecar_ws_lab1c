import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations

def transform_to_matrix(transform):
    """将 TransformStamped 消息转换为 4x4 numpy 矩阵"""
    trans = transform.transform.translation
    rot = transform.transform.rotation
    translation_matrix = tf_transformations.translation_matrix([trans.x, trans.y, trans.z])
    rotation_matrix = tf_transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    return np.dot(translation_matrix, rotation_matrix)

def matrix_to_transform(matrix, stamp, parent_frame, child_frame):
    """将 4x4 numpy 矩阵转换为 TransformStamped 消息"""
    trans = tf_transformations.translation_from_matrix(matrix)
    rot = tf_transformations.quaternion_from_matrix(matrix)
    
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]
    return t

class DynamicTFCamPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')
        
        # TF 监听器和广播器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 预计算从 base_link 到相机的静态变换矩阵
        # 左相机: y = +0.05
        self.T_base_to_left = tf_transformations.translation_matrix([0.0, 0.05, 0.0])
        # 右相机: y = -0.05
        self.T_base_to_right = tf_transformations.translation_matrix([0.0, -0.05, 0.0])

        # 创建一个定时器来周期性地查找和发布TF
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # 1. 获取 odom -> base_link 的当前变换
            t_odom_to_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform odom to base_link: {ex}')
            return
            
        # 2. 将获取的变换转换为4x4矩阵
        T_odom_to_base = transform_to_matrix(t_odom_to_base)
        
        # 3. 计算 odom -> left_cam 的变换
        # T_odom_left = T_odom_base * T_base_left
        T_odom_to_left = np.dot(T_odom_to_base, self.T_base_to_left)
        
        # 4. 计算 left_cam -> right_cam 的变换
        # T_left_right = T_left_base * T_base_right
        # 其中 T_left_base 是 T_base_left 的逆矩阵
        T_left_to_base = np.linalg.inv(self.T_base_to_left)
        T_left_to_right = np.dot(T_left_to_base, self.T_base_to_right)
        
        # 获取当前时间戳
        current_stamp = self.get_clock().now().to_msg()

        # 5. 将计算出的矩阵转换回 TransformStamped 消息并广播
        # odom -> left_cam
        t_odom_to_left = matrix_to_transform(
            T_odom_to_left, current_stamp, 'odom', 'left_cam')
            
        # left_cam -> right_cam
        t_left_to_right = matrix_to_transform(
            T_left_to_right, current_stamp, 'left_cam', 'right_cam')
        
        self.tf_broadcaster.sendTransform([t_odom_to_left, t_left_to_right])

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFCamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
