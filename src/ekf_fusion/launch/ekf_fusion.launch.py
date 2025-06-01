import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # UWB Tracking Node
        Node(
            package="uwb_tracking_ros2",
            executable="uwb_tracking_dwm1001",
            name="uwb_tracking",
            output="screen"
        ),

        # Include the VectorNav launch file instead of treating it as an executable
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("vectornav"), "launch", "vectornav.launch.py")
            )
        ),

        # EKF Fusion Node
        Node(
            package="ekf_fusion",
            executable="ekf_fusion_node",
            name="ekf_fusion",
            output="screen",
            parameters=[{
                "uwb_topic": "/dwm1001/id_1B12/pose_kf",
                "imu_topic": "/vectornav/imu_uncompensated",
                "output_velocity_topic": "/velocity/world"
            }]
        )
    ])

