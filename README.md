# mobile_robot

如何启动物理机器人

ros2 launch robot_bringup robot.launch.py

如何启动仿真环境

单一机器人环境
ros2 launch robot_gazebo robot_gazebo.launch.py

多机器人环境
ros2 launch robot_gazebo robots_gazebo.launch.py

启动仿真建图

ros2 launch robot_cartographer cartographer.launch.py use_sim_time:=true

保存栅格地图

ros2 run nav2_map_server map_saver_cli -f ~/map

保存pbstream文件

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/porizou/map.pbstream'}"



参考链接
    https://qiita.com/porizou1/items/4ddeb5a0aa6c62a4c294
    https://qiita.com/porizou1/items/70cd05be433ad02b7423
    