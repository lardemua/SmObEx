# SmObEx: Smart Object Exploration

My Mechanical Engineering Masters Thesis in the field of Robotis.

## Thesis title

Smart object exploration by robotic manipulator

## Advisor

Miguel Riem de Oliveira [GitHub](https://github.com/miguelriemoliveira/)

Department of Mechanical Engineering, University of Aveiro

LAR: Laboratory of Automation and Robotics

# Completed tasks

- [x] Retrived Point Cloud from camera and visualized in Rviz
- [x] Single view OctoMap visualization in Rviz
- [x] Installed ROS Industrial on the Fanuc m6ib/6s
- [x] Connected ROS to the Fanuc
- [x] Updated the Fanuc package
     - [x] Updated the xacro file
     - [x] Updated the stl
     - [x] Create the support package 
     - [x] Create MoveIt "package"
- [ ] Calibrate Xtion

# Built with

- [ROS](http://www.ros.org/)
- [OpenNi 2](http://wiki.ros.org/openni2_launch/)
- [Octomap Server](http://wiki.ros.org/octomap_server)
- [FANUC Driver](http://wiki.ros.org/fanuc) (based on)

## Hardware

Fanuc Robot M6iB/6S

Asus Xtion PRO LIVE

# Installation guides

## ROS Industrial and FANUC

```
sudo apt-get install ros-melodic-industrial-core

cd ~/catkin_ws

git clone https://github.com/ros-industrial/abb ./src/abb

catkin_make --pkg abb

git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git ./src/universal_robot

rosdep update

<!-- rosdep install --from-paths src --ignore-src --rosdistro melodic -->

catkin_make

sudo apt-get install ros-melodic-ros-canopen
```

Ah this stage, ROS Industrial is installed. Now the FANUC part.

```
git clone -b indigo https://github.com/ros-industrial/fanuc.git

rosdep install --from-paths src --ignore-src --rosdistro melodic
```

Now you must go to /src/fanuc and only leave the followin folders (where it's m6ib, it should be your robot model):

- fanuc
- fanuc_driver
- fanuc_m6ib_moveit_config
- fanuc_m6ib_moveit_plugins
- fanuc_m6ib_support
- fanuc_resources
- LICENSE
- readme.md

If you try to compile now it won't work, what you need to do is to follow [this issues' intructions](https://github.com/ros-industrial/fanuc/issues/241) and make changes on the fanuc_m6ib_moveit_plugins/m6ib_kinematics/src/fanuc_m6ib_manipulator_ikfast_moveit_plugin.cpp.

After this run ```catkin_make``` to try everything.

## Aruco Hand Eye

```
cd ~/catkin_ws/src

git clone https://github.com/pal-robotics/aruco_ros

git clone https://github.com/lagadic/vision_visp

git clone https://github.com/jhu-lcsr/aruco_hand_eye

cd ..

catkin_make
```

# See the robot

Everything is based on [this tutorial](http://wiki.ros.org/fanuc/Tutorials/Running).

## Moving the joint with the interface

```
roslaunch fanuc_m6ib_support test_m6ib6s.launch
```

If the graphics aren't right, the solution is [on this issue](https://github.com/ros-visualization/rviz/issues/1249#issuecomment-403351217). So all you have to do is

```
export LC_NUMERIC="en_US.UTF-8"

roslaunch fanuc_m6ib_support test_m6ib.launch
```
## Moving the robot with the TP

Wire connect the robot to the machine.

Set the IP of the machine to 192.168.0.200

On the TP, run rosstate

On the Linux machine, run ```roslaunch fanuc_m6ib_support robot_state_visualize_m6ib6S.launch robot_ip:=192.168.0.230```