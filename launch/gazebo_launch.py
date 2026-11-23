from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from xmobot_launch.model import model_actions


def generate_launch_description():
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    return LaunchDescription([
        *model_actions(use_sim_time='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(gazebo / 'launch' / 'gazebo.launch.py'))),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-topic', 'robot_description', '-entity', 'xmobot'], output='screen'),
        Node(package='controller_manager', executable='spawner.py',
             arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
             output='screen'),
    ])
