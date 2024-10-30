FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "bin/bash", "-c" ]

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

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN cd /home && mkdir catkin_ws && cd catkin_ws && mkdir src \
    && source /opt/ros/noetic/setup.bash && catkin_make \
    && echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

COPY rooster_2020-03-10-10-36-30_0.bag /home/rooster_2020-03-10-10-36-30_0.bag

COPY lidar_node /home/catkin_ws/src/lidar_node   

RUN source /opt/ros/noetic/setup.bash && source /home/catkin_ws/devel/setup.bash \
    && cd /home/catkin_ws \
    && catkin_make > build.log 2>&1 || (cat build.log && exit 1)

# Set the entrypoint to bash
ENTRYPOINT [ "bin/bash", "-c", "source /opt/ros/noetic/setup.bash && roscore" ]