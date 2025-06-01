import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
from collections import deque

class PoseFusionNode(Node):
    def __init__(self):
        super().__init__('odom_fusion_node')

        # Subscribers
        self.position_sub = message_filters.Subscriber(self, PoseStamped, '/dwm1001/id_D63A/pose_kf')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/vectornav/imu_uncompensated')

        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.position_sub, self.imu_sub], queue_size=100, slop=0.01
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/uwb_pose', 10)

        # Store last position & time for velocity computation
        self.prev_position = None
        self.prev_time = None

        # Moving average filter for linear velocity (reduce noise)
        self.velocity_window = deque(maxlen=25)  # Stores last 25 velocity estimates

        self.get_logger().info("Odom Fusion Node Started...")

    def synchronized_callback(self, position_msg: PoseStamped, imu_msg: Imu):
        """Callback when both position and IMU messages are received"""
        self.get_logger().info("Received position and IMU message!")

        # Extract position
        x, y, z = position_msg.pose.position.x, position_msg.pose.position.y, position_msg.pose.position.z
        current_time = position_msg.header.stamp.sec + position_msg.header.stamp.nanosec * 1e-9  # Convert to seconds

        # Extract orientation (from IMU)
        qx, qy, qz, qw = imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w

        # Compute linear velocity if previous position is available
        linear_velocity_x, linear_velocity_y = 0.0, 0.0
        if self.prev_position is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                vx = (x - self.prev_position[0]) / dt
                vy = (y - self.prev_position[1]) / dt

                # Apply moving average filter for velocity smoothing
                self.velocity_window.append((vx, vy))
                smoothed_vx = np.mean([v[0] for v in self.velocity_window])
                smoothed_vy = np.mean([v[1] for v in self.velocity_window])
            else:
                smoothed_vx, smoothed_vy = 0.0, 0.0

            linear_velocity_x, linear_velocity_y = smoothed_vx, smoothed_vy

        # Update previous position & time
        self.prev_position = (x, y)
        self.prev_time = current_time

        # Extract angular velocity from IMU (gyroscope data)
        angular_velocity_x = imu_msg.angular_velocity.x
        angular_velocity_y = imu_msg.angular_velocity.y
        angular_velocity_z = imu_msg.angular_velocity.z

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

        # Set linear velocity
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.linear.y = linear_velocity_y
        odom_msg.twist.twist.linear.z = 0.0  # Assuming 2D motion

        # Set angular velocity (from IMU gyroscope)
        odom_msg.twist.twist.angular.x = angular_velocity_x
        odom_msg.twist.twist.angular.y = angular_velocity_y
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Publish Odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published Odometry: Position ({x}, {y}, {z}), Linear Vel ({linear_velocity_x}, {linear_velocity_y}), Angular Vel ({angular_velocity_x}, {angular_velocity_y}, {angular_velocity_z})")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

