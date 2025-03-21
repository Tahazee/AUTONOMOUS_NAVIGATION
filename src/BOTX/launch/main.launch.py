import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch the simulation
        ExecuteProcess(
            cmd=["ros2", "launch", "BOTX", "launch_sim.launch.py", "world:=src/BOTX/worlds/house.world"],
            output="screen"
        ),

        # Launch RViz2 with config file
        ExecuteProcess(
            cmd=["rviz2", "-d", "src/BOTX/config/nav.rviz"],
            output="screen"
        ),

        # Launch SLAM Toolbox in async mode
        ExecuteProcess(
            cmd=["ros2", "launch", "slam_toolbox", "online_async_launch.py", "params_file:=./src/BOTX/config/mapper_params_online_async.yaml", "use_sim_time:=true"],
            output="screen"
        ),

        # Launch Nav2 for navigation
        ExecuteProcess(
            cmd=["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=true"],
            output="screen"
        ),

        # Launch teleoperation node
        ExecuteProcess(
            cmd=["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args", "-r", "/cmd_vel:=/diff_cont/cmd_vel_unstamped"],
            output="screen"
        ),
    ])
