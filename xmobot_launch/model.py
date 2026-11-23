from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def publish_description(context, model_args):
    path = LaunchConfiguration('urdf_file').perform(context)
    mappings = {name: LaunchConfiguration(name).perform(context) for name in model_args}
    document = xacro.process_file(path, mappings=mappings)
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(document.toxml(), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )]


def model_actions(use_sim_time):
    share = Path(get_package_share_directory('xmobot'))
    model = share / 'urdf' / 'xmobot.urdf.xacro'
    # Read defaults from the installed model so both launchers expose the same parameters.
    arguments = ET.parse(model).getroot().findall('{http://www.ros.org/wiki/xacro}arg')
    defaults = {arg.attrib['name']: arg.attrib['default'] for arg in arguments}
    defaults['control_config'] = str(share / 'config' / 'ros2_control.yaml')
    return [
        DeclareLaunchArgument('urdf_file', default_value=str(model)),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=publish_description, args=[list(defaults)]),
    ]
