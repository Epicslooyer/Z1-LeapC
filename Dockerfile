#Setup ros-noetic
FROM ghcr.io/sloretz/ros:noetic-simulators-osrf
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

#Validate keys and curl
RUN apt-get update -o Acquire::AllowInsecureRepositories=true && \
    apt-get install -y curl
    
RUN sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sudo rm -f /etc/apt/sources.list.d/ros-latest.list


RUN apt-get update && apt-get install -y ros-noetic-moveit \
    ros-noetic-ros-controllers ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools terminator python3-pip && rm -rf /var/lib/apt/lists/*

RUN pip install flask flask-ask-sdk ask-sdk 
WORKDIR /home/${USER}

#Setup catkin workspace

#Setup z1_unitree

#Setup leap motion