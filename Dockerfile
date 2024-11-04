# Use the official Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment variables to non-interactive to avoid any prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary tools and dependencies
RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set the locale
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add the ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add the ROS 2 repository
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 packages and X11 dependencies
RUN apt-get update && \
    apt-get install -y \
    ros-iron-desktop \
    libgl1-mesa-glx \
    x11-apps && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
    ros-iron-rviz2 \
    ros-iron-imu-tools &&\ 
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
# Source the ROS 2 setup script
RUN echo "source /opt/ros/iron/setup.bash" >> /etc/bash.bashrc


RUN apt-get update && \
    apt-get install -y \
    python3-smbus\    
    python3-colcon-common-extensions\ 
    python3-pip && \
    pip3 install pyserial &&\
    pip3 install scipy  &&\
    pip3 install numpy  &&\
    pip3 install transforms3d &&\
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
    ros-iron-tf-transformations\
    ros-iron-plotjuggler-ros\
    ros-iron-imu-transformer&&\    
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gpiod \
    libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
        iproute2 \
        can-utils\
	udev




RUN usermod -aG dialout ${USERNAME} 



COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]

COPY setup_terminal.sh /usr/local/bin/
RUN echo "source /usr/local/bin/setup_terminal.sh" >> /root/.bashrc

WORKDIR /ros2_ws







