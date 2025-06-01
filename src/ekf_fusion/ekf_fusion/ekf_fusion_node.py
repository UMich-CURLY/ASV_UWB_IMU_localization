import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped

class EKFFusionNode(Node):
    def __init__(self):
        super().__init__("ekf_fusion")

        # Parameters
        self.declare_parameter("uwb_topic", "/uwb/position")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("output_velocity_topic", "/velocity/world")

        # Subscribe to UWB and IMU
        self.uwb_sub = self.create_subscription(PoseStamped, 
                                                self.get_parameter("uwb_topic").value, 
                                                self.uwb_callback, 10)

        self.imu_sub = self.create_subscription(Imu, 
                                                self.get_parameter("imu_topic").value, 
                                                self.imu_callback, 10)

        # Publisher for velocity in world frame
        self.vel_pub = self.create_publisher(TwistStamped, 
                                             self.get_parameter("output_velocity_topic").value, 
                                             10)

        # State Variables
        self.prev_time = None
        self.prev_pos = None
        self.vel = np.array([0.0, 0.0])

        # Kalman Filter Matrices
        self.X = np.zeros((4, 1))  # State: [x, y, v_x, v_y]
        self.P = np.eye(4) * 1.0   # Covariance Matrix
        self.A = np.eye(4)         # Transition Model
        self.Q = np.eye(4) * 0.01  # Process Noise
        self.R = np.eye(2) * 0.1   # Measurement Noise (UWB)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Measurement Model

    def uwb_callback(self, msg):
        if self.prev_time is None:
            self.prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.prev_pos = np.array([msg.pose.position.x, msg.pose.position.y])
            return

        # Compute dt
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = curr_time - self.prev_time
        if dt <= 0:
            return

        # Compute UWB velocity estimate
        pos = np.array([msg.pose.position.x, msg.pose.position.y])
        vel_uwb = (pos - self.prev_pos) / dt

        # Update UWB measurement
        Z = np.array([[pos[0]], [pos[1]]])

        # Kalman Update
        Y = Z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.X += K @ Y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        # Update time and position
        self.prev_time = curr_time
        self.prev_pos = pos

    def imu_callback(self, msg):
        # Get acceleration from IMU
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        # Convert acceleration to world frame using IMU orientation
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        r = R.from_quat(quat)
        acc_world = r.apply(acc)

        # Compute dt
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            return

        dt = curr_time - self.prev_time
        if dt <= 0:
            return

        # Update transition model with dt
        self.A[0, 2] = dt
        self.A[1, 3] = dt

        # Predict Step
        self.X = self.A @ self.X
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Update velocity estimate with acceleration
        self.X[2] += acc_world[0] * dt
        self.X[3] += acc_world[1] * dt

        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.twist.linear.x = self.X[2, 0]
        vel_msg.twist.linear.y = self.X[3, 0]
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

