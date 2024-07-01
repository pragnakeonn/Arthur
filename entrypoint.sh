#!/bin/bash

# set -e
#
# nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
# DISPLAY=:1.0
# export DISPLAY

#export ROS_MASTER_URI=http://$(hostname --ip-address):11311
#export ROS_HOSTNAME=$(hostname --ip-address)
#ros2 run param_test_pkg test_node --ros-args --params-file ~/catkin_ws/src/param_test_pkg/config/params.yaml

#source $HOME/.cargo/env
source /opt/ros/humble/setup.bash
cd colcon_ws/
source install/local_setup.bash
#source "${HOME}/create3_ws/install/local_setup.bash"
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export GAZEBO_MODEL_PATH=~/create3_ws/install/realsense2_description/share/realsense2_description/meshes/:$GAZEBO_MODEL_PATH
export IGNITION_VERSION=fortress
#sudo docker exec -it create3_humble_container /bin/bash
#ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
#ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
# To use the rust version of relaxedik
# roslaunch relaxed_ik launcher_race_rust.launch
# To use the rust version of relaxedik and run rviz to test it
# roslaunch relaxed_ik launcher_race_rust_rviz.launch
# -v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/create3_sim_bckp/create3_lfd/src:/root/catkin_ws/src/create3_lfd_lidar/create3_lfd_lidar/src/ \
# To use the python version of relaxedik
# roslaunch relaxed_ik launcher_race_python.launch
# To use the python version of relaxedik and run rviz to test it
#roslaunch relaxed_ik launcher_race_python_rviz.launch
#Classic gazebo, empty world
#ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
#ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py
#Ignition gazebo
#ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
#ros2 launch irobot_create_ignition_bringup create3_ignition.launch.py
#<xacro:include filename="$(find irobot_create_description)/urdf/sensors/lidar.urdf.xacro" />
exec "$@"
#git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
#ros2 launch realsense2_camera rs_launch.py filters:=pointcloud align_depth:=true
#Found remap rule '~/out:=scan'. This syntax is deprecated. Use '--ros-args --remap ~/out:=scan' instead.
#-v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/create3_sim/:/root/create3_ws/src/create3_sim/ \
#-v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/realsense_gazebo_plugin/:/root/create3_ws/src/realsense_gazebo_plugin/ \
#Found multiple nodes with same name: /laser_controller. This might be due to multiple plugins using the same name. Try changing one of the the plugin names or use a different ROS namespace. This error might also result from a custom plugin inheriting from another gazebo_ros plugin and the custom plugin trying to access the ROS node object hence creating multiple nodes with same name. To solve this try providing the optional node_name argument in gazebo_ros::Node::Get() function. 
#[ERROR] [gzserver-1]: process has died [pid 56, exit code -11, cmd 'gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so extra-gazebo-args --ros-args --params-file /root/create3_ws/install/irobot_create_gazebo_bringup/share/irobot_create_gazebo_bringup/config/gazebo_params.yaml'].
#ros2 run tf2_tools view_frames
#ros2 run rqt_gui rqt_gui
#sudo docker cp create3_humble_container:/root/catkin_ws/frames_2023-12-20_20.29.58.pdf .
#ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
#in_map_pose = do_transform_pose(msg.pose, trans)
#ros2 launch create3_lidar_slam slam_toolbox_launch.py use_sim_time:=True
#ros2 launch create3_lfd_bringup lfd_launch.py use_sim_time:=True
#ros2 launch create3_lfd_bringup tf2_launch.py use_sim_time:=True
#docker system prune
#-v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/create3_lfd:/root/catkin_ws/src/create3_lfd \
#```bash
#docker login
#./build-image.sh create3-galactic
#docker tag create3-galactic irobotedu/create3-galactic:0.0.1
#docker push irobotedu/create3-galactic:0.0.1
#docker tag create3-galactic irobotedu/create3-galactic:latest
#docker push irobotedu/create3-galactic:latest
#```
### private ssh key ###3
## ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
### change owner of lock sign folder
# id
# sudo chown -R 'uid':'gid' <folder_name>
#sudo chown -R '1000':'1000' example_interfaces/
#-v /etc/localtime:/etc/localtime:ro \
#-v /home/pragna/ARTHUR/MRS/system_intallation/create3-docker/create3-humble/create3_lfd/:/root/catkin_ws/src/create3_lfd/ \
#create3-humble
#python3 -c "import sys; print(sys.path)" | grep lfd_services
# pose:{data:[5,4,3]}}
#ros2 service call /lfd_srv lfd_interfaces/srv/LfdRes "{pose:{data:[5,4,3]}, angle: 2.5}"
#ros2 run teleop_twist_keyboard teleop_twist_keyboard
#ros-humble-nav-msgs \
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 # add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# 
# ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>