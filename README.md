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

## Installation

### 1. Clone the Repository

Clone the repository to your local machine:

```bash
git clone https://github.com/your-username/BOTX.git
cd BOTX
## 2. Setup ROS2 Environment

Ensure you have **ROS2** installed. For installation instructions, refer to the [ROS2 installation guide](https://docs.ros.org/en/foxy/Installation.html).

## 3. Install Dependencies

To install all required dependencies, run:

```bash
sudo apt update
sudo apt install ros-<ros2-distro>-slam-toolbox ros-<ros2-distro>-navigation2 ros-<ros2-distro>-robot-state-publisher
