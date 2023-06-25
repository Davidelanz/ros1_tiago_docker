FROM ubuntu:bionic
LABEL org.opencontainers.image.description "Docker enabled TIAGo++ ROS workspace"

WORKDIR /root/catkin_ws
ENV DEBIAN_FRONTEND noninteractive

# Use bash in Dockerfile (https://stackoverflow.com/a/39777387)
SHELL ["/bin/bash", "-c"]

# Guide to create TIAGo++ workspace:
# http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS

# Set up ROS packages sources
RUN apt-get update -y \
    && apt-get install -y curl lsb-release gnupg \
    # setup your sources.list:
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    # set up your keys:
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ROS packages installation
RUN apt-get update -y \
    && apt-get install -y \
    git \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    ros-melodic-control-toolbox \
    ros-melodic-controller-manager \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-desktop-full \
    ros-melodic-joint-state-controller \
    ros-melodic-joy ros-melodic-joy-teleop \
    ros-melodic-moveit-commander \
    ros-melodic-moveit-core \
    ros-melodic-moveit-kinematics \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-ros-perception \
    ros-melodic-moveit-ros-planning-interface \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-navigation \
    ros-melodic-ompl \
    ros-melodic-sound-play \
    ros-melodic-teleop-tools \
    ros-melodic-twist-mux \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Append the following command to ~/.bashrc to allow any console to use the catkin commands:
RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc

# Create the workspace for TIAGo++
RUN mkdir -p /root/catkin_ws \
    && cd /root/catkin_ws \
    # download tiago_dual_public-melodic.rosinstall:
    && wget https://raw.githubusercontent.com/pal-robotics/tiago_dual_tutorials/master/tiago_dual_public-melodic.rosinstall \
    # clone all the required repositories within the workspace:
    && rosinstall src /opt/ros/melodic tiago_dual_public-melodic.rosinstall

# Set up rosdep and install TIAGo++ additional dependencies
RUN source /opt/ros/melodic/setup.bash \
    && rosdep init \
    && rosdep fix-permissions \
    && rosdep update --rosdistro melodic \
    && apt-get update -y \
    && rosdep install -y --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup slam_toolbox joint_impedance_trajectory_controller cartesian_impedance_controller omni_base_description omni_drive_controller pal_moveit_capabilities pal_moveit_plugins  pal_loc_measure pal_map_manager ydlidar_ros_driver" \
    && rm -rf /var/lib/apt/lists/*

# Proceed building the workspace
RUN source /opt/ros/melodic/setup.bash \
    && catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)

# Append the following command to ~/.bashrc to always source catkin_ws
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
