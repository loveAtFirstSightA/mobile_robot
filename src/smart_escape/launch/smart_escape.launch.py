import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('smart_escape'),
        'rviz',
        'rviz.rviz')
    return LaunchDescription([
        Node(
            package='smart_escape',
            executable='smart_escape',
            # parameters=[os.path.join(get_package_share_directory('/* param_pkg_name */'),
            #     'config', '/* file_name */')],
            output='screen'),
        Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                output='screen')     
    ])
