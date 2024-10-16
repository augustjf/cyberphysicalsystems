FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# Install ROS Noetic dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["bash", "-c"/bin/bash"", "roscore & bash"]