import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_robot_description(urdf_file, namespace):
    return Command([
        'xacro ', urdf_file,
        ' robot_namespace:=', namespace
    ])


def generate_launch_description():
    pkg_robot_desc = get_package_share_directory('robot_description')
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # --------------------
    # Launch 参数声明
    # --------------------
    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    world_file = LaunchConfiguration('world')

    declare_urdf_file_cmd = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(pkg_robot_desc, 'urdf', 'robot.xacro'),
        description='URDF/Xacro 文件路径'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        name='robot_namespace',
        default_value='',  # default none
        description='机器人命名空间（也用于 xacro 中的 robot_namespace 参数）'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_robot_gazebo, 'worlds', 'house.world'),
        description='Gazebo world 文件路径'
    )

    # --------------------
    # Robot 描述和节点
    # --------------------
    robot_description = get_robot_description(urdf_file, robot_namespace)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # # 增加机器人位姿发布
    # robot_true_pose = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('robot_true_pose'),
    #         'launch', 'robot_true_pose.launch.py'
    #         )
    #     )
    # )

    # --------------------
    # Gazebo 启动
    # --------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # --------------------
    # 实体生成（spawn）
    # --------------------
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_namespace,
            '-x', '-1.0',
            '-y', '4.0',
            # '-Y', '-1.5708'
            '-Y', '0'
        ],
        output='screen'
    )

    # --------------------
    # 返回 LaunchDescription
    # --------------------
    return LaunchDescription([
        declare_urdf_file_cmd,
        declare_use_sim_time_cmd,
        declare_robot_namespace_cmd,
        declare_world_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node
        # robot_true_pose
    ])
