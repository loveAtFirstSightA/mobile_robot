import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import TimerAction
import time

def generate_launch_description():
    """
    生成用于 Gazebo 仿真环境并加载多个机器人模型的 LaunchDescription。
    支持通过 xacro 使用命名空间参数，适用于 ROS 2 Humble。
    """
    # 获取相关共享目录路径
    pkg_share_dir = get_package_share_directory('robot_description')
    gazebo_share_dir = get_package_share_directory('robot_gazebo')

    # 声明 launch 参数
    urdf_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(pkg_share_dir, 'urdf', 'robot.xacro'),
        description='机器人 URDF/Xacro 文件路径。'
    )
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='是否启用仿真时间。'
    )
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(gazebo_share_dir, 'worlds', 'empty_world.world'),
        description='Gazebo 世界文件路径。'
    )

    # 获取参数配置
    urdf_file = LaunchConfiguration('urdf_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # 启动 Gazebo 服务端和客户端
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'paused': 'false'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            )
        )
    )

    # # 增加机器人位姿发布
    # robot_true_pose = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('robot_true_pose'),
    #         'launch', 'robot_true_pose.launch.py'
    #         )
    #     )
    # )
    
    # 机器人列表与初始位姿配置
    robot_names = ['R1', 'R2', 'R3', 'R4', 'R5',
                   'R6', 'R7', 'R8', 'R9', 'R10'
    ]
    initial_poses = [
        {'x': '0.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-0.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '1', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-1', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '1.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-1.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},  
        {'x': '2', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-2', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '2.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-2.5', 'y': '1', 'z': '0.1', 'yaw': '-1.5708'}
    ]

    robot_groups = []

    for i, robot_name in enumerate(robot_names):
        # 1. 生成 robot_description 内容（带命名空间参数）
        robot_description = Command([
            'xacro ', urdf_file,
            TextSubstitution(text=' robot_namespace:='),
            TextSubstitution(text=robot_name)
        ])

        # 2. 创建 robot_state_publisher 节点
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'frame_prefix': robot_name + '/'
            }],
            output='screen'
        )
        
        # # 3. 创建 joint_state_publisher 节点
        # jsp_node = Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # )

        # 3. 创建 spawn_entity 节点 - 添加延迟以避免同时生成所有机器人
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', f'/{robot_name}/robot_description',
                '-entity', robot_name,
                '-x', initial_poses[i]['x'],
                '-y', initial_poses[i]['y'],
                '-z', initial_poses[i]['z'],
                '-Y', initial_poses[i]['yaw']
            ],
            output='screen'
        )
        
        # 添加延迟生成 - 每个机器人间隔1秒生成
        delayed_spawn = TimerAction(
            period=1.0 * i,  # 每个机器人间隔1秒
            actions=[spawn_node],
        )

        # 4. 组合进 GroupAction
        robot_group = GroupAction([
            rsp_node,
            # jsp_node,
            delayed_spawn  # 使用延迟后的生成动作
        ])

        robot_groups.append(robot_group)

    # 构建 LaunchDescription
    launch_items = [
        urdf_arg,
        sim_time_arg,
        world_arg,
        gzserver,
        gzclient
        # robot_true_pose
    ] + robot_groups

    return LaunchDescription(launch_items)
