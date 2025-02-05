FROM balenalib/amd64-ubuntu:jammy-20241112

#### DO NOT EDIT THIS SECTION ####
# ROS2 CORE
# ----------------------------------------------

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./config/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]

# ROS2 BASE
# ----------------------------------------------

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
#### DO NOT EDIT ABOVE SECTION ####

# EDIT THIS SECTION ####
# ----------------------------------------------

# use bash instead of sh
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive


# RUN mkdir -p /etc/apt/keyrings \
#     && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
#     && apt-get update  && apt-get install apt-transport-https \
#     && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list \
#     && apt-get update 
    
# RUN apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
#     libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
#     cmake g++

# RUN apt-get install --yes librealsense2-dkms=2.54.1
# RUN apt-get install --yes librealsense2-utils
# RUN apt-get install --yes librealsense2-dev
# RUN apt-get install --yes librealsense2-dbg

RUN apt-get update && apt-get upgrade -y && apt-get dist-upgrade -y
RUN apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
                       git wget cmake build-essential \
                       libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
                       libudev-dev v4l-utils

RUN git clone https://github.com/IntelRealSense/librealsense.git /librealsense
WORKDIR /librealsense
RUN scripts/setup_udev_rules.sh
# RUN scripts/patch-realsense-ubuntu-lts-hwe.sh

RUN mkdir build
WORKDIR /librealsense/build
RUN cmake ../
RUN make uninstall && make clean && make && make install


RUN sudo apt-get update && \
    sudo apt-get install -y \
    ros-humble-realsense2-camera \ 
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# copy ROS packages into container
COPY antdrone_bringup /ros2_ws/src/antdrone_bringup
COPY antdrone_realsense /ros2_ws/src/antdrone_realsense

# build ROS packages and allow non-compiled
# sources to be edited without rebuild
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

# add sourcing ros2_ws to entrypoint
RUN sed --in-place --expression \
    '$isource "/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

# add packages to path
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# start udev to trigger rules to add devices
RUN sed --in-place \
    '/^exec "\$@"/i /lib/systemd/systemd-udevd --daemon && udevadm trigger' \
    /ros_entrypoint.sh