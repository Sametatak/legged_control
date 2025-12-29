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

### installation
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
Clone the repository:

Bash

git clone https://github.com/Sametatak/legged_control.git)
Build and Source:

Bash

cd ~/ros2_ws
colcon build
source install/setup.bash
Usage
### To launch the simulation environment along with the leg control logic, run the following command:

Bash

ros2 launch go2_config legged_control.launch.py

