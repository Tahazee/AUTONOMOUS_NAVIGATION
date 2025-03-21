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




### Setup ROS2 Environment

Ensure you have ROS2 installed. For installation instructions, refer to the ROS2 installation guide.

### 3. Install Dependencies

To install all required dependencies, run:

```bash

sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-robot-state-publisher
```
Additionally, install any Python dependencies (if applicable):

```bash
pip install -r requirements.txt 
```
### 4. Build the Workspace

If you have a ROS2 workspace, build the workspace to compile the URDF model and ROS2 packages:

```bash
colcon build --symlink-install
```
Source the ROS2 workspace:
```bash
source install/setup.bash
```
### 5. Launch the System

To launch the system, run the following ROS2 launch command:

```bash
ros2 launch BOTX main.launch.py
```
This will bring up the robot model, sensors, and navigation stack. You can use RViz to visualize the robot's sensor data and its navigation path.

### 6. Run the Robot in Simulation (Optional)

You can also run the robot in a simulated environment such as Gazebo for testing and development purposes. To launch the robot in Gazebo, run:

```bash
ros2 launch BOTX launch_sim.launch.py
```
## Usage

### Autonomous Navigation

Once the system is up and running, the robot will perform autonomous navigation tasks using the **Nav2 stack** for path planning, local and global navigation, and obstacle avoidance.

-   The **LiDAR** sensor provides distance measurements and environmental data for SLAM and mapping.
-   The **depth camera** and **RGB camera** offer visual data for object detection, localization, and mapping.
-   **SLAMToolbox** uses sensor data to build and update the map in real-time.

The robot autonomously plans its path based on the environment and performs collision avoidance in dynamic environments.

### Controlling the Robot

You can interact with the robot using various ROS2 tools:

-   **RViz**: Visualize the robot's sensors, path planning, and SLAM data.
-   **Teleoperation**: Use `teleop_twist_keyboard` or another teleoperation package to manually control the robot.

To teleoperate the robot:

```bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Customizing the Robot

The robot's URDF model can be customized for different sensors, actuators, or additional hardware. You can modify the URDF file and recompile the system to suit your needs.

## Project Structure

```bash
BOTX/
├── CMakeLists.txt
├── config
│   ├── diff_drive.rviz
│   ├── empty.yaml
│   ├── final_nav.rviz
│   ├── mapper_params_online_async.yaml
│   ├── my_controllers.yaml
│   ├── nav.rviz
│   ├── peaksimulation.rviz
│   ├── twist_mux.yaml
│   └── view_bot.rviz
├── description
│   ├── camera.xacro
│   ├── depth.xacro
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot_core.xacro
│   ├── robot.urdf.xacro
│   └── ros2_control.xacro
├── launch
│   ├── launch_sim.launch.py
│   ├── main.launch.py
│   ├── __pycache__
│   │   ├── launch_sim.launch.cpython-310.pyc
│   │   └── rsp.launch.cpython-310.pyc
│   └── rsp.launch.py
├── LICENSE.md
├── package.xml
├── README.md
└── worlds
    ├── empty.world
    ├── house.world
    ├── obs2.world
    ├── obs.world
    └── wall.world

```

## Contributing

Contributions to **BOTX** are welcome! To contribute:

1.  Fork the repository.
2.  Create a new branch (`git checkout -b feature-branch`).
3.  Make your changes and commit them (`git commit -am 'Add new feature'`).
4.  Push to your forked repository (`git push origin feature-branch`).
5.  Submit a pull request.

## Acknowledgements

-   **ROS2**: For providing the platform for developing robotic systems.
-   **SLAMToolbox**: For real-time SLAM and mapping.
-   **Nav2**: For autonomous navigation and path planning.
-   **Gazebo**: For simulation of robot dynamics and sensor integration.
-   **OpenCV** and **PCL**: For computer vision and point cloud processing.
