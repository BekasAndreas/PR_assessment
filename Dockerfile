FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Set timezone and locales
RUN ln -fs /usr/share/zoneinfo/Europe/Athens /etc/localtime && \
    apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=${LC_ALL} LANG=${LANG}

# Install basic tools
RUN apt-get update && \
    apt-get install -y \
        software-properties-common \
        curl \
        lsb-release \
        gnupg2 && \
    rm -rf /var/lib/apt/lists/*

# Add ROS 2 apt source
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                            | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update

# Install ROS 2 and required packages
RUN apt-get install -y \
        ros-humble-desktop-full \
        ros-dev-tools \
        libeigen3-dev \
        libyaml-cpp-dev \
        ros-humble-rviz-visual-tools && \
    rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create ROS2 workspace directory
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws