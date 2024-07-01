FROM  osrf/ros:humble-desktop-full as base-ros
LABEL maintainer="pdas@keonn.com"

#######
# NVIDIA setup
#######
ENV NVIDIA_VISIBLE_DEVICES \
	${NVIDIA_VISIBLE_DEVICES:-all}

ENV NVIDIA_DRIVER_CAPABILITIES \
	${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
LABEL com.nvidia.volumes.needed="nvidia_driver"
# working directory
ENV HOME /root
WORKDIR ${HOME}

ENV DEBIAN_FRONTEND=noninteractive
#ENV TZ=America/Los_Angeles
#RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Change shell to bash
SHELL ["/bin/bash", "-c"]

# Make apt to always ignore recommended and suggested packages
# This is particularly important with rosdep which invoked apt without `--no-install-recommends`
RUN echo \
  'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";' > /etc/apt/apt.conf.d/01norecommend
#RUN apt install ros-some-pkg-required-by-some-other-pkg
RUN rosdep update

# Upgrade everything
RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

#######
# Install Gazebo classic
#######

RUN curl -sSL http://get.gazebosim.org | sh


#######
# Install Gazebo ignition fortress
#######

RUN apt-get update && apt-get install -y wget git && rm -rf /var/lib/apt/lists/*
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
        ignition-fortress \
        ros-humble-ros-ign-bridge \
        ros-humble-ros-ign-gazebo \
        ros-humble-ros-ign-interfaces \
        ros-humble-irobot-create-msgs \
        ros-humble-ign-ros2-control \
        ros-humble-joint-state-broadcaster \
        ros-humble-rplidar-ros \
        ros-humble-gazebo-dev \
        ros-humble-ros2-controllers \
        ros-humble-slam-toolbox \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-plugins \
        ros-humble-joint-state-publisher \
        ros-humble-gazebo-ros2-control \
        ros-humble-tf-transformations \
        ros-humble-teleop-twist-keyboard \
        ros-humble-robot-localization \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-slam-toolbox \
        ros-humble-rqt* \
        ros-humble-nav-2d-msgs \
    && rm -rf /var/lib/apt/lists/*
#RUN apt-get update && apt-get install -y ros-humble-ros2bag \
#        ros-humble-rosbag2-converter-default-plugins \
#        ros-humble-rosbag2-storage-default-plugins \

RUN apt-get install -y build-essential python3-colcon-common-extensions python3-rosdep ros-humble-rmw-fastrtps-cpp 
### Realsense setup ###
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update
RUN apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
RUN apt-get install -y ros-humble-realsense2-camera
# Install ROS dev tools
#######
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
        git \
        python3-pip \
        python3-dev \
        python3-opencv \
        libglib2.0-0
# Install python3 modules for collision
RUN apt-get update && apt-get install -y python3 python3-pip \
    python3-rospkg-modules \
    python3-empy && \    
    python3 -m pip install pandas numpy scipy tqdm matplotlib seaborn sympy
RUN pip3 install torch torchvision torchaudio -f https://download.pytorch.org/whl/cu111/torch_stable.html
RUN pip install setuptools==56.2.0

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils

ENV HOME /root
WORKDIR ${HOME}
COPY ./shell_setup.sh ${HOME}/
RUN ["chmod","+x","/root/shell_setup.sh"]
RUN apt-get install ros-humble-realsense2-camera
RUN cd ${HOME} && mkdir -p colcon_ws

RUN cd colcon_ws && mkdir -p src
WORKDIR ${HOME}/colcon_ws
#RUN colcon build --symlink-install
RUN echo ' \n\
source /opt/ros/humble/setup.bash \n\
source /root/colcon_ws/install/local_setup.bash' >> ${HOME}/.bashrc
RUN source ${HOME}/.bashrc
RUN cd ${HOME}/colcon_ws/src
WORKDIR ${HOME}/colcon_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
RUN git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git -b foxy-devel
RUN git clone -b humble  https://github.com/iRobotEducation/irobot_create_msgs.git

RUN pip3 install transforms3d pandas setuptools==58.2.0
RUN mkdir -p create3_sim
COPY ./create3_sim_bckp ${HOME}/colcon_ws/src/create3_sim

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p start_create3
COPY ./start_create3 ${HOME}/colcon_ws/src/start_create3

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p lfd_interfaces
COPY ./lfd_interfaces ${HOME}/colcon_ws/src/lfd_interfaces

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p lfd_services
COPY ./lfd_services ${HOME}/colcon_ws/src/lfd_services

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p smc_services
COPY ./smc_services ${HOME}/colcon_ws/src/smc_services

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p lfd_pose_package
COPY ./lfd_pose_package ${HOME}/colcon_ws/src/lfd_pose_package

RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p navigation2
COPY ./navigation2-humble ${HOME}/colcon_ws/src/navigation2


RUN cd ${HOME}/colcon_ws/src
RUN mkdir -p nav2_rl_api
COPY ./nav2_rl_api ${HOME}/colcon_ws/src/nav2_rl_api


RUN vcs import ~/colcon_ws/src/ < ~/colcon_ws/src/create3_sim/irobot_create_gazebo/demo.repos


RUN cd ${HOME}/colcon_ws
WORKDIR ${HOME}/colcon_ws
RUN apt-get install python3-rosdep -y
RUN apt-get update \
    && rosdep install --from-path src -yi --rosdistro humble --skip-keys=librealsense2 -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c '. /opt/ros/humble/setup.sh; colcon build --symlink-install'
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    && colcon test



#######
# Common utilities
#######

RUN apt-get update && apt-get install -y \
        iputils-ping \
        nano \
        vim \
    && rm -rf /var/lib/apt/lists/*

#######
# Env setup
#######

WORKDIR ${HOME}
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp " >> ${HOME}/.bashrc
RUN echo ' \n\
export IGNITION_VERSION=fortress \n\
source /opt/ros/humble/setup.bash \n\
source /root/colcon_ws/install/local_setup.bash' >> ${HOME}/.bashrc


RUN source ${HOME}/.bashrc

COPY entrypoint.sh /
EXPOSE 11311

ENTRYPOINT ["bash","/entrypoint.sh"]
CMD ["bash"]
