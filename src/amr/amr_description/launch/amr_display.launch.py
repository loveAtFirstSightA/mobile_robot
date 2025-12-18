import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace", default="amr")

    # 获取功能包共享目录，用于定位 URDF/Xacro 和 RViz 配置文件
    pkg_share_dir = get_package_share_directory('amr_description')

    # ---------- Launch 参数声明 ----------
    # URDF/Xacro 文件路径参数
    urdf_file = LaunchConfiguration('urdf_file')
    declare_urdf_file_cmd = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(pkg_share_dir, 'urdf', 'amr.xacro'),
        description='URDF/Xacro 文件的路径'
    )

    # 是否启用 joint_state_publisher_gui（GUI 控制关节）
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')
    declare_use_jsp_gui_cmd = DeclareLaunchArgument(
        name='use_jsp_gui',
        default_value='true',
        description='是否启用 joint_state_publisher_gui'
    )

    # 是否使用仿真时间（通常用于 Gazebo 或模拟环境）
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='是否启用仿真时间'
    )

    # RViz 配置文件路径参数
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(pkg_share_dir, 'rviz', 'display.rviz'),
        description='RViz 配置文件路径'
    )

    # ---------- 机器人描述生成 ----------
    # 使用 xacro 命令生成 robot_description 字符串
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' robot_namespace:=', robot_namespace
    ])

    # ---------- 节点定义 ----------
    # 无 GUI 模式的关节状态发布器节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_jsp_gui)
    )

    # GUI 模式的关节状态发布器节点（提供图形界面手动调整关节）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_jsp_gui)
    )

    # 机器人状态发布器节点：发布 TF 和机器人的 URDF 描述
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # RViz 可视化节点：加载配置并显示机器人模型
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 返回完整的 LaunchDescription
    return LaunchDescription([
        declare_urdf_file_cmd,
        declare_use_jsp_gui_cmd,
        declare_use_sim_time_cmd,
        declare_rviz_config_file_cmd,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
