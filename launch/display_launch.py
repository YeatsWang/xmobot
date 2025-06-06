import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('robot_length', default_value='0.5'))
    ld.add_action(DeclareLaunchArgument('robot_width', default_value='0.4'))
    ld.add_action(DeclareLaunchArgument('robot_height', default_value='0.1'))
    ld.add_action(DeclareLaunchArgument('wheel_radius', default_value='0.08'))
    ld.add_action(DeclareLaunchArgument('wheel_width', default_value='0.04'))
    ld.add_action(DeclareLaunchArgument('wheel_separation_x', default_value='0.5'))
    ld.add_action(DeclareLaunchArgument('wheel_separation_y', default_value='0.4'))
    ld.add_action(DeclareLaunchArgument('drive_type', default_value='4wis'))
    ld.add_action(DeclareLaunchArgument('use_mesh', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_arms', default_value='true'))

    # Paths and substitutions
    pkg_share = get_package_share_directory('xmobot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'xmobot.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' robot_length:=', LaunchConfiguration('robot_length'),
            ' robot_width:=', LaunchConfiguration('robot_width'),
            ' robot_height:=', LaunchConfiguration('robot_height'),
            ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
            ' wheel_width:=', LaunchConfiguration('wheel_width'),
            ' wheel_separation_x:=', LaunchConfiguration('wheel_separation_x'),
            ' wheel_separation_y:=', LaunchConfiguration('wheel_separation_y'),
            ' drive_type:=', LaunchConfiguration('drive_type'),
            ' use_mesh:=', LaunchConfiguration('use_mesh'),
            ' use_arms:=', LaunchConfiguration('use_arms')
        ]),
        value_type=str
    )

    # Nodes
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    ))
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    ))
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'xmobot_display.rviz')]
    ))

    return ld
