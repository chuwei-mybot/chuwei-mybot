'''
FileName: 
Description: 
Autor: Liujunjie/Aries-441
StudentNumber: 521021911059
Date: 2022-11-13 19:34:39
E-mail: sjtu.liu.jj@gmail.com/sjtu.1518228705@sjtu.edu.cn
LastEditTime: 2022-11-14 19:48:30
'''
'''
FileName: 
Description: 
Autor: Liujunjie/Aries-441
StudentNumber: 521021911059
Date: 2022-11-13 19:34:39
E-mail: sjtu.liu.jj@gmail.com/sjtu.1518228705@sjtu.edu.cn
LastEditTime: 2022-11-13 20:27:01
'''
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('tribot'))
    xacro_file = os.path.join(pkg_path,'urdf','tribot_gazebo.xacro')
    robot_description_config = xacro.process_file(xacro_file)


    urdf_file_name = 'tribot_description.urdf'
    urdf = os.path.join(pkg_path,
       'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    # Create a joint_state_publisher node
    #params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        #output='screen',
        #condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
        #parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher,
        joint_state_publisher_node
    ])