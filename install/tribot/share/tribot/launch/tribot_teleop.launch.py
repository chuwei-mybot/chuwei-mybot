'''
FileName: 
Description: 
Autor: Liujunjie/Aries-441
StudentNumber: 521021911059
Date: 2022-11-14 20:22:53
E-mail: sjtu.liu.jj@gmail.com/sjtu.1518228705@sjtu.edu.cn
LastEditTime: 2022-11-14 20:55:16
'''
#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import rclpy
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    #params = {'scale_linear',0.1 ,'scale_angular',0.4}
    kinematic = Node(package='tribot', executable='tribot_teleop',
                        output='screen')
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        kinematic
    ])