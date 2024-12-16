FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-humble-nav2-bringup \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-image-proc \
    ros-humble-depth-image-proc \
    python3-networkx \
    ros-humble-imu-tools \
    ros-humble-joy-linux \
    ros-humble-laser-filters \
    libglvnd0 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    x11-xserver-utils \
    ros-humble-robot-localization \
    ros-humble-rosbridge-server \
    ros-humble-domain-bridge \
    && rm -rf /var/lib/apt/lists/*
    

RUN pip install setuptools==58.2.0

WORKDIR /home/humble_ws

COPY ./linorobot2_navigation /home/humble_ws/src/linorobot2_navigation
COPY ./linorobot2_localization /home/humble_ws/src/linorobot2_localization

# Build the workspace and source the setup files
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/humble_ws/install/setup.bash" >> ~/.bashrc

CMD bash -c "source /home/humble_ws/install/setup.bash && \
            ros2 launch linorobot2_navigation navigation.launch.py WHEEL_ODOMETRY:=${WHEEL_ODOMETRY} DRONE_NAME:=${DRONE_NAME}"