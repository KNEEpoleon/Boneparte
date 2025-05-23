# Use CUDA image as base
FROM nvidia/cuda:12.2.2-cudnn8-devel-ubuntu22.04

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
# RUN mkdir /ros_ws
# RUN cd ros_ws/
WORKDIR /ros_ws
# RUN mkdir -p /ros_ws/src/Perception && \
#     /ros_ws/src/Manipulation && \
#     /ros_ws/src/Planning

# COPY src /ros_ws/src
COPY src/perception /ros_ws/src/perception
COPY src/manipulation /ros_ws/src/manipulation
COPY src/vision_pro /ros_ws/src/vision_pro

# Install kernel headers.  Get the kernel version from 'uname -r'.
# RUN apt-get update && apt-get install -y linux-headers-$(uname -r)

# Install essential dependencies and common tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    gnupg2 \
    curl \
    build-essential \
    tzdata \
    ranger \
    git \
    neovim \
    iputils-ping \
    net-tools \
    dnsutils \
    htop \
    nvtop \
    wget \
    unzip \
    gnome-terminal \
    tmux \
    locales \
    apt-transport-https && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt-get install apt-transport-https \
mkdir -p /etc/apt/keyrings \
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp |  tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
tee /etc/apt/sources.list.d/librealsense.list \
apt-get update \
apt-get install librealsense2-dkms \
apt-get install librealsense2-utils \
apt-get update


# Setup ROS 2 Humble repository and install ROS 2 Humble (Desktop Full Installation)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ament-cmake \
    python3-rosdep2 \
    python3-pip \
    python3-nose \
    python3-colcon-common-extensions \
    python3-vcstool \
    python-is-python3
# Install ROS 2 RealSense package separately
RUN apt-get update && apt-get install -y ros-humble-realsense2-camera \
    ros-humble-librealsense2*

# Install ROS 2 control, controllers, and visualization tools
RUN apt-get install -y \ 
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-graph-msgs \
    ros-humble-realtime-tools

# Install MoveIt and Gazebo ROS 2 control
RUN apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-visual-tools \
    ros-humble-pick-ik \
    vim-gtk3 \
    ament-cmake-nose

# Install torch
RUN pip install torch==2.3.1+cu121 torchvision==0.18.1+cu121 torchaudio==2.3.1+cu121 --index-url https://download.pytorch.org/whl/cu121


# Install x11 dependencies for GUI applications
RUN apt-get install -y libgl1-mesa-glx libxrender1 libxext6 libxtst6

# Copy the source code for Segment Anything
COPY src/perception/segment-anything-2/ /ros_ws/src/perception/segment-anything-2/
RUN cd /ros_ws/src/perception/segment-anything-2 && pip install -e ".[demo]"

# Copy the source code for CoTracker
# COPY src/perception/co-tracker/ /ros_ws/src/perception/co-tracker/
# RUN cd /ros_ws/src/perception/co-tracker && pip install -e .

# Additonal Python Libraries (To override the SAM2 versions)
RUN pip install open3d==0.18.0 \
    opencv-python==4.9.0.80 \
    transforms3d==0.4.2 \
    numpy==1.24.2 \
    pyserial \
    imageio \
    transitions \
    questionary

# Initialize rosdep
RUN rosdep update

# Install oh-my-bash
RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)" && \
    sed -i 's/OSH_THEME="font"/OSH_THEME="robbyrussell"/' ~/.bashrc && \
    sed -i \
    's|PS1="${arrow}  ${python_venv}${base_directory} "|PS1="[docker] ${arrow}  ${python_venv}${base_directory} "|' \
    /root/.oh-my-bash/themes/robbyrussell/robbyrussell.theme.sh

# Copy custom commands into .bashrc
COPY docker/commands.bashrc /ros_ws/
RUN cat /ros_ws/commands.bashrc >> ~/.bashrc

# Set the default command to run when starting the container
COPY docker/entrypoint.sh /ros_ws/
ENTRYPOINT ["./entrypoint.sh"]
CMD ["/bin/bash"]