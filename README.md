# create3-docker

Dockerfile for Running Generative AI based Motion trajectory for Create3 

Each Docker image should be placed, together with all the additional files it requires, in a directory named as the image itself.

### Deploying the container to machine

# - Build image locally
```bash
./build-image.sh IMAGE_DIR
```
Example for build
./build-image.sh create3-humble
# - Run the image
Run the image with 
./run-image.sh

# - Launch the usual blank gazebo setup 

ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py

or 
# - Launch aws scene 
ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py

# - Start another docker session 
sudo docker exec -it create3_humble_container /bin/bash

# - source 

source /opt/ros/humble/setup.bash
cd colcon_ws/
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# - After full exploration of the simulation scene, undock the robot
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"

# - Dock the robot
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"

# - Load sensor data in rviz
In rviz window with odom frame as fixed frame, first Add topic Laserscan /scan, then /depth /points
P.N.- Add the laserscan sensor first in rviz before starting slam toolbox, otherwise, the map will not load in rviz after starting slam toolbox
# - Star the slam tool box
ros2 launch create3_lidar_slam slam_toolbox_launch.py

# - To load map in rviz
Change fixed frame to map, now add /map topic
The first map should load
# - To save map in a file
ros2 run nav2_map_server map_saver_cli -f my_map

# - Using generative AI
We will not teleoperate neither use NAV2 to move the robot to create the map, the robot will move automatically using LFD launc files. LFD uses map to base-link transform to get current robot posaition and then predicts the next position
this position is transformed to /odom frame 
the robot navigates to this odometry location using /navigate_to_position client (provided by irobot)



# - Start navigation 
ros2 launch nav2_bringup navigation_launch.py

# ros2 launch path_planner_server navigation.launch.py

nav2_bringup bringup_launch.py with autostart enabled, map file is needed, one way is to save map file while slam is running and use that map file

# - Start lfd pose generator
# - make sure in rviz map frame is selected as global frame
# ros2 launch lfd_pose_package lfd_pose_pkg.launch.py

# - Start nav2 rl api 
ros2 launch nav2_rl_api nav2_rl.launch.py

# - record lasercan
ros2 bag record /scan 



ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true,goal_pose:{pose:{position:{x: -4.23328,y: -7.79357,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.99, w: 0.05}}}}"

x: 0.0
      y: 0.0
      z: 0.9986627298381786
      w: 0.05169866567095499



 

source ~/.bashrc