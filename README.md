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




> Written w-   **Nav2 Stack**: Install via  `apt`  or build from source.
    
    bash
    
    Copy
    
    sudo apt install ros-humble-nav2-bringup
    
-   **Gazebo**  (Optional): For simulation purposes.
    
    bash
    
    Copy
    
    sudo apt install ros-humble-gazebo-ros-pkgs
    

## Installation

1.  **Clone the Repository**:
    
    bash
    
    Copy
    
    git clone https://github.com/your-username/autonomous-navigation-botx.git
    cd autonomous-navigation-botx
    
2.  **Build the Workspace**:
    
    bash
    
    Copy
    
    colcon build
    source install/setup.bash
    
3.  **Install Dependencies**:
    
    bash
    
    Copy
    
    rosdep install --from-paths src --ignore-src -r -y
    

## Usage

### Running SLAM

To start SLAM and create a map of the environment:

bash

Copy

ros2 launch botx_navigation slam_launch.py

### Autonomous Navigation

Once the map is created, you can start autonomous navigation:

bash

Copy

ros2 launch botx_navigation navigation_launch.py

### Simulation (Optional)

If you want to test BotX in a simulated environment using Gazebo:

bash

Copy

ros2 launch botx_gazebo gazebo_launch.py

## Configuration

The configuration files for SLAM and Nav2 are located in the  `config`  directory. You can modify parameters such as:

-   **Map resolution**
    
-   **Robot footprint**
    
-   **Costmap parameters**
    
-   **Planner settings**
    

Refer to the  [Nav2 documentation](https://navigation.ros.org/)  for detailed information on configuration options.

## Contributing

We welcome contributions to the BotX Autonomous Navigation project! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request.

1.  Fork the repository.
    
2.  Create a new branch (`git checkout -b feature/YourFeatureName`).
    
3.  Commit your changes (`git commit -m 'Add some feature'`).
    
4.  Push to the branch (`git push origin feature/YourFeatureName`).
    
5.  Open a pull request.
    

## License

This project is licensed under the  **MIT License**. See the  [LICENSE](https://license/)  file for details.

## Acknowledgments

-   **ROS2 Community**: For providing an excellent framework for robotics development.
    
-   **SLAM Toolbox Developers**: For their robust SLAM implementation.
    
-   **Nav2 Team**: For their comprehensive navigation stack.
    
-   **Gazebo**: For providing a powerful simulation environment.
    

----------

**Happy Navigating!**  🚀ith [StackEdit](https://stackedit.io/).
