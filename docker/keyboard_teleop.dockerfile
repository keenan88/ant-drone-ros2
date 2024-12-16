FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    gnome-terminal \
    dbus-x11 \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
WORKDIR /root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
