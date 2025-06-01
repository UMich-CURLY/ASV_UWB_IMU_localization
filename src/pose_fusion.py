import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters

class PoseFusionNode(Node):
    def __init__(self):
        super().__init__('pose_fusion_node')

        # Subscribers
        self.position_sub = message_filters.Subscriber(self, PoseStamped, '/dwm1001/id_D63A/pose_kf')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/vectornav/imu_uncompensated')

        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.position_sub, self.imu_sub], queue_size=100, slop=0.5
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/uwb_pose', 10)

        self.get_logger().info("Pose Fusion Node Started...")

    def synchronized_callback(self, position_msg: PoseStamped, imu_msg: Imu):
        """Callback when both position and IMU messages are received"""
        self.get_logger().info("Received position and IMU message!")

        # Extract data
        x, y, z = position_msg.pose.position.x, position_msg.pose.position.y, position_msg.pose.position.z
        qx, qy, qz, qw = imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = position_msg.header.frame_id  # Assuming same frame
        odom_msg.child_frame_id = "base_link"  # Adjust if needed

        # Set pose
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Publish Odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published Odometry: Position ({x}, {y}, {z}) Orientation ({qx}, {qy}, {qz}, {qw})")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

