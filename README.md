# SmObEx: Smart Object Exploration

My Mechanical Engineering Masters Thesis in the field of Robotis.

## Thesis title

Smart object exploration by robotic manipulator

## Advisor

Miguel Riem de Oliveira [GitHub](https://github.com/miguelriemoliveira/)

Department of Mechanical Engineering, University of Aveiro

LAR: Laboratory of Automation and Robotics

# Built with

- [ROS](http://www.ros.org/)
- [OpenNi 2](http://wiki.ros.org/openni2_launch/)
- [Octomap Server](http://wiki.ros.org/octomap_server)

## Hardware

Fanuc Robot M6iB/6S

Asus Xtion PRO LIVE

# ROS Industrial and FANUC installation guide

```
sudo apt-get install ros-melodic-industrial-core

cd ~/catkin_ws

git clone https://github.com/ros-industrial/abb ./src/abb

catkin_make --pkg abb

git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git ./src/universal_robot

rosdep update

rosdep install --from-paths src --ignore-src --rosdistro melodic

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

If you try to compile now it won't work, what you need to do is to follow [this issues' intructions](https://github.com/ros-industrial/fanuc/issues/241).

After this run ```catkin_make``` to try everything.