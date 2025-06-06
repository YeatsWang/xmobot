import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ------------------------------------------------------------------------
    # 1. 声明 Launch 参数
    # ------------------------------------------------------------------------
    # urdf_file：默认指向 xmobot/urdf/robot.urdf.xacro
    urdf_file_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('xmobot'),
            'urdf',
            'xmobot.urdf.xacro'
        ]),
        description='Absolute path to robot URDF XACRO file'
    )

    # use_sim_time：默认在 Gazebo 中使用仿真时间
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ------------------------------------------------------------------------
    # 2. 通过 xacro 将 URDF 拷贝成纯文本字符串
    # ------------------------------------------------------------------------
    # Command(...) 会在运行时调用 xacro 可执行文件，把 LaunchConfiguration('urdf_file') 所指路径下的 xacro 文件“编译”成完整 URDF 字符串
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        LaunchConfiguration('urdf_file')
    ])

    # 必须使用 ParameterValue 将 robot_description_content 明确标记为字符串类型，避免被当作 YAML 去解析
    robot_description_param = ParameterValue(
        robot_description_content,
        value_type=str
    )

    # ------------------------------------------------------------------------
    # 3. joint_state_publisher 和 robot_state_publisher 节点
    # ------------------------------------------------------------------------
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            # 以纯文本形式传 robot_description
            {'robot_description': robot_description_param},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # ------------------------------------------------------------------------
    # 4. 启动 Gazebo 空世界
    # ------------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # ------------------------------------------------------------------------
    # 5. spawn_entity：把机器人模型插入到 Gazebo
    # ------------------------------------------------------------------------
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  # 从 robot_state_publisher 发布的 /robot_description 话题读取 URDF
            '-entity', 'xmobot',            # 在 Gazebo 中的实体名字，可自行修改
            # 如果需要设置初始位置，可以附加 -x -y -z，例如：
            # '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # ------------------------------------------------------------------------
    # 6. 返回 LaunchDescription
    # ------------------------------------------------------------------------
    return LaunchDescription([
        # 声明所有 launch 参数
        urdf_file_arg,
        use_sim_time_arg,

        # joint_state_publisher
        joint_state_publisher_node,

        # robot_state_publisher
        robot_state_publisher_node,

        # 启动 Gazebo
        gazebo_launch,

        # 在 Gazebo 中 spawn 机器人
        spawn_entity_node,
    ])
