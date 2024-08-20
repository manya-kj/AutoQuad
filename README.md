# **AutoQuad-ROS2-NAV**
AutoQuad-ROS2-NAV : An Autonomous Indoor Navigation system with Quadcopter

![WhatsApp Image 2024-08-19 at 18 32 28_584c0308](https://github.com/user-attachments/assets/2d45c6c8-62b1-4feb-b296-e1e4d149ac03)

## **Project Overview**
This repository contains the necessary ROS2 packages for the AutoQuad, an autonomous quadcopter designed for indoor navigation. 
The provided picture is the quadcopter and its orientation on the RViz window. Following are the packages with brief overview of it:
- ```uav_drone_bringup ``` Contains the necessary launch files and configurations for launching of Gazebo window with drone spawned inside the world and Rviz window spawning the drone with its orientation shown.
- ```uav_drone_control ``` Defines the teleop key, drone controls and autonomous explorer algorithm and files
- ```uav_drone_description ``` Describes the drone models and urdf files, alone with gazebo world file.
- ```uav_drone_simulation ``` Implements the autonomous navigation algorithm with the corresponding launch files.

These packages represent the core components of the AutoQuad's functionality, demonstrating practical applications of ROS2 in robotic indoor navigation.

## **Getting Started**
### **Prerequites**
- ROS2 Humble : [Documentation](https://docs.ros.org/en/humble/index.html)
- Python 3
- Gazebo : [Installation](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
- ROS2 Packages

## **Installation**
1.Create a ROS2 workspace:
```
mkdir ~/your_workspace-name_ws
cd ~/your_workspace-name_ws
```
2.Clone this repository onto your workspace:
```
git clone https://github.com/manya-kj/quadcopter.git
```
3. **Build and Source** your workspace:
```
colcon build
source install/setup.bash
```

## **Usage**
### **Manual Control**
This launches the Gazebo window with quadcopter spawned inside the world, Rviz window with quadcopter along with its orientation, and teleop window.
Teleop window is used to control the drone, type in the commands shown on the teleop window to maneuver the drone accordingly.
```
ros2 launch uav_drone_bringup uav_drone_bringup.launch.py
```
![WhatsApp Image 2024-08-19 at 20 58 55_a28fa17e](https://github.com/user-attachments/assets/2951a704-3bb7-4142-b498-47972cc3f57a)

### **Autonomous Navigation**
This launches the Gazebo window with quadcopter spawned inside the world, Rviz window with quadcopter along with its orientation, and teleop window.
Now the teleop window is used only for the quadcopter's takeoff, once it has taken off it will autonomously navigate around the gazebo world.
```
ros2 launch uav_drone_simulation autonomous_exploration.launch.py
```
![WhatsApp Image 2024-08-19 at 18 26 11_3ad6c568](https://github.com/user-attachments/assets/cb70d747-99b5-48ce-bfe5-68e95d95e50d)
![https://github.com/user-attachments/assets/97d2c43c-f8c8-4142-856f-7fdd280d14fc]
