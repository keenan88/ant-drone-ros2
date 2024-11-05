FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

#RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select antworker_bringup antworker_pcl

ENV GAZEBO_MODEL_PATH=/home/humble_ws/src/

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc