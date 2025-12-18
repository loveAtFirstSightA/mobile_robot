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
    pkg_share_dir = get_package_share_directory('amr_description')
    gazebo_share_dir = get_package_share_directory('amr_gazebo')

    # 声明 launch 参数
    urdf_arg = DeclareLaunchArgument(
        name='urdf_file',
        default_value=os.path.join(pkg_share_dir, 'urdf', 'amr.xacro'),
        description='机器人 URDF/Xacro 文件路径。'
    )
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='是否启用仿真时间。'
    )
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(gazebo_share_dir, 'worlds', 'warehouse.world'),
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

    # 增加机器人位姿发布
    amr_true_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('amr_true_pose'),
            'launch', 'amr_true_pose.launch.py'
            )
        )
    )
    
    # 机器人列表与初始位姿配置
    robot_names = ['amr_0', 'amr_1', 'amr_2', 'amr_3', 'amr_4',
                   'amr_5', 'amr_6', 'amr_7', 'amr_8', 'amr_9'
                   
                #    'amr_10', 'amr_11', 'amr_12', 'amr_13', 'amr_14',
                #    'amr_15', 'amr_16', 'amr_17', 'amr_18', 'amr_19'
                   
                #    'amr_20', 'amr_21', 'amr_22', 'amr_23', 'amr_24',
                #    'amr_25', 'amr_26', 'amr_27', 'amr_28', 'amr_29',
                   
                #    'amr_30', 'amr_31', 'amr_32', 'amr_33', 'amr_34',
                #    'amr_35', 'amr_36', 'amr_37', 'amr_38', 'amr_39'
    ]
    initial_poses = [
        {'x': '1', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-1', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '3', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-3', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '5', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-5', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},  
        {'x': '7', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-7', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '9', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'},
        {'x': '-9', 'y': '21.0', 'z': '0.1', 'yaw': '-1.5708'}
        
        # {'x': '1', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '-1', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '3', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '-3', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '5', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '-5', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '7', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '-7', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '9', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'},
        # {'x': '-9', 'y': '-21.0', 'z': '0.1', 'yaw': '1.5708'}
        
        # {'x': '-15', 'y': '1.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '-1.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '3.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '-3.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '5.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '-5.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '7.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '-7.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '9.0', 'z': '0.1', 'yaw': '0.0'},
        # {'x': '-15', 'y': '-9.0', 'z': '0.1', 'yaw': '0.0'},
        
        # {'x': '15', 'y': '1.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '-1.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '3.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '-3.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '5.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '-5.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '7.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '-7.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '9.0', 'z': '0.1', 'yaw': '3.142'},
        # {'x': '15', 'y': '-9.0', 'z': '0.1', 'yaw': '3.142'}
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
        gzclient,
        amr_true_pose
    ] + robot_groups

    return LaunchDescription(launch_items)
