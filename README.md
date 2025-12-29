# Legged Control

This repository contains the control software and simulation configurations for the Unitree Go2 quadruped robot.

> **Project Context:** This project was developed as the final assignment for the **KON440E Legged Robotics** course at **Istanbul Technical University (ITU)**.

## Requirements
Unitree Go2 Robot Description
This project requires the URDF and description files from the official Unitree repository.

### System Requirements
* **ROS 2 Humble** (Desktop Full recommended)
* **Gazebo**

### ROS 2 Dependencies
Install the required packages using the following commands:

sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control


### To launch the simulation environment along with the leg control logic, run the following command:


ros2 launch go2_config legged_control.launch.py

