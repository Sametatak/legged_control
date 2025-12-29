# Legged Control

This repository contains the control software and simulation configurations for the Unitree Go2 quadruped robot.

> **Project Context:** This project was developed as the final assignment for the **KON440E Legged Robotics** course at **Istanbul Technical University (ITU)**.

## Requirements

### System Requirements
* **ROS 2 Humble** (Desktop Full recommended)
* **Gazebo**

### ROS 2 Dependencies
Install the required packages using the following commands:

```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
Unitree Go2 Robot Description
This project requires the URDF and description files from the official Unitree repository. The robot description (URDF) is derived from:

Unitree Robotics ROS 2 Repository

Please ensure you satisfy the requirements listed in the Unitree repo and have the description package available in your workspace.

Installation
Follow these steps to set up the workspace and build the project:

Create and go to workspace:

Bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
Clone the repository:

Bash

git clone [https://github.com/Sametatak/legged_control.git](https://github.com/Sametatak/legged_control.git)
Build and Source:

Bash

cd ~/ros2_ws
colcon build
source install/setup.bash
Usage
To launch the simulation environment along with the leg control logic, run the following command:

Bash

ros2 launch go2_config legged_control.launch.py
This launch file will:

Start the Gazebo simulation.

Load the Go2 robot model (URDF).

Initialize the necessary controllers and the legged control node.

Acknowledgments
Unitree Robotics: For providing the Go2 robot description and URDF files.

Istanbul Technical University: For the KON440E course resources and guidance
