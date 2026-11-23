from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from xmobot_launch.model import model_actions


def generate_launch_description():
    share = Path(get_package_share_directory('xmobot'))
    return LaunchDescription([
        *model_actions(use_sim_time='false'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'xmobot_display.rviz')]),
    ])
