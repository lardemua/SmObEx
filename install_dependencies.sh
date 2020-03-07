#!/bin/bash

cd ~/catkin_ws/src
sudo apt install -y ros-melodic-industrial-core
sudo apt install -y ros-melodic-ros-canopen
sudo apt install -y ros-melodic-moveit-visual-tools
sudo apt install -y ros-melodic-moveit-ros-planning-interface
sudo apt install -y ros-melodic-openni2-launch
sudo apt install -y ros-melodic-octomap-server
sudo apt install -y ros-melodic-pcl-ros
sudo apt install -y ros-melodic-vision-visp
git clone https://github.com/ros-industrial/fanuc.git
git clone https://github.com/pal-robotics/aruco_ros
git clone https://github.com/jhu-lcsr/aruco_hand_eye
git clone https://github.com/tu-darmstadt-ros-pkg/hector_models
git clone https://github.com/miguelriemoliveira/octomap_tools
cd ..
catkin_make
rospack profile