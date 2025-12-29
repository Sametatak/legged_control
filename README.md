# Legged Control

This repository contains the control software and simulation configurations for the Unitree Go2 quadruped robot.

> **Project Context:** This project was developed as the final assignment for the **KON440E Legged Robotics** course at **Istanbul Technical University (ITU)**.

## Requirements

### Software
* **ROS 2 Humble** (Desktop Full recommended)
* **Gazebo**

### Dependencies
Install the required ROS 2 packages using the following commands:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros2-control \
                 ros-humble-xacro \
                 ros-humble-robot-localization \
                 ros-humble-ros2-controllers \
                 ros-humble-ros2-control
