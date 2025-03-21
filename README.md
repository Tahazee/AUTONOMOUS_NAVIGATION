# BOTX - Autonomous Navigation Robot with ROS2

## Overview

**BOTX** is a custom URDF-based robot model designed for autonomous navigation and sensor integration. The robot is equipped with a variety of sensors, including a depth camera, a regular camera, and LiDAR. It utilizes **ROS2** for its control and communication stack, with **SLAMToolbox** for simultaneous localization and mapping (SLAM) and the **Nav2 stack** for autonomous navigation.

BOTX is a fully autonomous system capable of performing complex navigation tasks in unknown environments, making use of real-time mapping, path planning, and obstacle avoidance. It is an ideal platform for research and development in robotic autonomy, computer vision, and SLAM technologies.

## Features

- **Custom URDF Robot Model**: Designed with a flexible and scalable URDF (Unified Robot Description Format) to accommodate various hardware configurations.
- **Sensor Integration**: Equipped with multiple sensors:
  - **Depth Camera**: For 3D perception and obstacle detection.
  - **RGB Camera**: For visual processing and environmental awareness.
  - **LiDAR**: For accurate distance measurements and 3D mapping.
- **ROS2 Integration**: Full ROS2 (Robot Operating System 2) support for easy communication and system integration.
- **Autonomous Navigation**:
  - **SLAMToolbox**: For real-time simultaneous localization and mapping.
  - **Nav2 Stack**: For path planning, obstacle avoidance, and autonomous navigation.
- **Modular Design**: The robot model and software are modular, allowing easy integration of additional sensors, actuators, or algorithms.
- **Simulation Support**: Can be used both in simulation (Gazebo, RViz) and on real hardware.

## Prerequisites

To run the **BOTX** system, ensure the following dependencies are installed:

- **ROS2** (Humble or later)
- **SLAMToolbox**
- **Nav2 Stack**
- **Python** (3.8 or later)
- **Gazebo** (For simulation)
- **RViz** (For visualization)
- **Additional ROS2 Packages**: See `requirements.txt` or package dependencies below.

### System Requirements

- Ubuntu 20.04 or 22.04 (Recommended for ROS2 compatibility)
- Compatible hardware (robot with LiDAR, depth camera, and RGB camera)
- GPU (recommended for running SLAM and perception algorithms efficiently)

Markdown

# Install Gazebo

Gazebo is a robot simulation environment that integrates seamlessly with ROS2. To install Gazebo, follow these steps:

## Install Gazebo:
```bash
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
Verify Gazebo installation by running:

Bash

gazebo
Install RViz2
RViz2 is a visualization tool for ROS2. It helps visualize robot sensor data, navigation paths, and more. To install RViz2, use the following command:

Bash

sudo apt install ros-<ros2-distro>-rviz2
Once installed, you can launch RViz2 using the following command:

Bash

ros2 run rviz2 rviz2
Clone and Setup the Project
Clone the repository for BOTX:

Bash

git clone <your-repository-url>
cd <repository-folder>
Build the Workspace
If you have a ROS2 workspace, build it to compile the URDF model and ROS2 packages:

Bash

colcon build --symlink-install
Source the workspace:

Bash

source install/setup.bash
Running the System
Launching the System
To launch the system, including the robot model, custom world, Gazebo simulation, and RViz2 visualization, run the following command:

Bash

ros2 launch botx_launch_file.launch.py
This command will do the following:

Start Gazebo: It will launch your custom robot in a custom world inside Gazebo.
Launch RViz2: The launch file also opens RViz2 with a pre-configured nav.rviz file for visualization.
Nav2 Stack: It will bring up the navigation stack with Nav2 for autonomous navigation, SLAMToolbox for mapping, and the robot's sensors.
Visualize in RViz2
Once the launch file is running, you can use RViz2 to visualize:

LiDAR Data: Real-time point cloud data for mapping and navigation.
Camera Data: RGB and depth images from the cameras.
SLAM and Path Planning: The robot’s map, localization, and navigation path will be displayed in RViz2.
Nav2: It will show the robot's path planning and navigation status.
Run the Robot in Simulation (Optional)
If you prefer running the robot in a simulated environment such as Gazebo, you can also launch the robot in Gazebo with the following command:

Bash

ros2 launch botx_gazebo.launch.py
Project Structure
The project is organized as follows:

Plaintext

BOTX/
├── urdf/                  # Custom URDF files for robot model
│   ├── robot_model.urdf
├── src/                   # Source code for the robot
│   ├── botx_navigation/   # Navigation-related packages
│   ├── botx_perception/   # Perception-related packages (e.g., camera, LiDAR)
│   ├── botx_control/      # Control and actuation packages
├── launch/                # ROS2 launch files
│   ├── botx_launch_file.launch.py
│   ├── botx_gazebo.launch.py
├── config/                # Configuration files (e.g., SLAM, Nav2)
│   ├── nav2_config.yaml
│   ├── slam_config.yaml
├── scripts/               # Helper scripts (e.g., for data collection, testing)
├── requirements.txt       # Python dependencies
└── README.md              # Project documentation
Usage
Autonomous Navigation
Once the system is running, the robot will perform autonomous navigation tasks using the Nav2 stack for path planning, local and global navigation, and obstacle avoidance.

LiDAR Sensor: Provides distance measurements and environmental data for SLAM and mapping.
Depth Camera and RGB Camera: Offer visual data for object detection, localization, and mapping.
SLAMToolbox: Uses sensor data to build and update the map in real-time.
Nav2 Stack: The robot autonomously plans its path and performs collision avoidance in dynamic environments.
Controlling the Robot
You can interact with the robot using various ROS2 tools:

RViz: Visualize the robot's sensors, path planning, and SLAM data.
Teleoperation: Use teleop_twist_keyboard or another teleoperation package to manually control the robot.
To teleoperate the robot:

Bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
Customizing the Robot
The robot's URDF model can be customized for different sensors, actuators, or additional hardware. You can modify the URDF file and recompile the system to suit your needs.

Contributing
Contributions to BOTX are welcome! To contribute:

Fork the repository.
Create a new branch (git checkout -b feature-branch).
Make your changes and commit them (git commit -am 'Add new feature').
Push to your forked repository (git push origin feature-branch).
Submit a pull request.
