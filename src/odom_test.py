import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R

class IMUFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')

        # Subscriber
        self.imu_sub = self.create_subscription(Imu, '/vectornav/imu_uncompensated', self.imu_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/imu_odometry', 10)

        # Store last orientation & time for angular velocity computation
        self.prev_orientation = None
        self.prev_time = None

        # Store initial IMU orientation for correction
        self.q_offset = None

        self.get_logger().info("IMU Fusion Node Started...")

    def imu_callback(self, imu_msg: Imu):
        self.get_logger().info("Received IMU message!")

        current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9

        # Extract orientation from IMU
        qx, qy, qz, qw = imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w
        q_imu = R.from_quat([qx, qy, qz, qw])

        # If first reading, compute the correction offset
        if self.q_offset is None:
            self.q_offset = q_imu.inv()  # Compute inverse for correction

        # Apply correction to align IMU to world frame
        q_corrected = self.q_offset * q_imu
        qx, qy, qz, qw = q_corrected.as_quat()  # Extract corrected quaternion
        
        current_orientation = np.array([qx, qy, qz, qw])

        # Compute angular velocity using orientation derivative
        angular_velocity_x, angular_velocity_y, angular_velocity_z = 0.0, 0.0, 0.0
        if self.prev_orientation is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                prev_rot = R.from_quat(self.prev_orientation)
                curr_rot = R.from_quat(current_orientation)
                rot_diff = curr_rot * prev_rot.inv()
                angular_velocity = rot_diff.as_rotvec() / dt  # Convert to angular velocity
                angular_velocity_x, angular_velocity_y, angular_velocity_z = angular_velocity

        # Update previous orientation & time
        self.prev_orientation = current_orientation
        self.prev_time = current_time

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "imu_link"
        odom_msg.child_frame_id = "imu"

        # Set orientation
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set angular velocity (computed from orientation change)
        odom_msg.twist.twist.angular.x = angular_velocity_x
        odom_msg.twist.twist.angular.y = angular_velocity_y
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Publish Odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published IMU Odometry: Orientation ({qx}, {qy}, {qz}, {qw}), Angular Vel ({angular_velocity_x}, {angular_velocity_y}, {angular_velocity_z})")


def main(args=None):
    rclpy.init(args=args)
    node = IMUFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

