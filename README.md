# ASV_UWB_IMU_localization

This repo is intended to combine UWB (Ultrawide-band) based 3d position estimate with the imu's orientation (we're using VN-100T which has n-built EKF running for orientation so pretty robust) to give out the pose and odom ros2 messages. The UWB based 3d position is completely taken from this repo: https://github.com/cliansang/uwb-tracking-ros/tree/ros2


## To build (after cloning):

colcon build --symlink-install
source install/setup.bash

## To launch:

### Publish transforms
cd src
python tf_pub.py

### Launch UWB
ros2 run uwb_tracking_ros2 uwb_tracking_dwm1001

### Launch IMU 
sudo chmod -R 777 /dev/ttyUSB0 (it's needed atleast for the VN-100T)
ros2 launch vectornav vectornav.launch.py

### Combine the UWB with IMU

- If velocity is also needed in addition to pose:
python odom_fusion_orient.py

or 

- If just pose is needed (not velocity):
python pose_fusion.py
