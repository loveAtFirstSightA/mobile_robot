#!/bin/bash


colcon build && source install/local_setup.bash && ros2 launch robot_gazebo robot_gazebo.launch.py
