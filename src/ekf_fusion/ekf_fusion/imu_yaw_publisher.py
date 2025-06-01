import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import tf_transformations as tf
import numpy as np

class IMUYawPublisher(Node):
    def __init__(self):
        super().__init__('imu_yaw_publisher')
        
        self.declare_parameter('world_frame_rotation', 0.0)  # Rotation angle in degrees
        self.world_frame_rotation = np.radians(self.get_parameter('world_frame_rotation').value)
        
        self.subscription = self.create_subscription(
            Imu,
            '/vectornav/imu',
            self.imu_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'imu/yaw', 10)
        
        self.get_logger().info("IMU Yaw Publisher Node has started.")
    
    def imu_callback(self, msg):
        # Extract quaternion
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        
        # Convert to Euler angles
        _, _, yaw = tf.euler_from_quaternion(quaternion)
        
        # Adjust yaw by rotating world frame
        adjusted_yaw = yaw - self.world_frame_rotation
        
        # Normalize yaw to [-pi, pi]
        adjusted_yaw = (adjusted_yaw + np.pi) % (2 * np.pi) - np.pi
        
        # Publish adjusted yaw
        yaw_msg = Float32()
        yaw_msg.data = adjusted_yaw
        self.publisher_.publish(yaw_msg)
        
        self.get_logger().info(f"Published yaw: {adjusted_yaw:.4f} rad")


def main(args=None):
    rclpy.init(args=args)
    node = IMUYawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

