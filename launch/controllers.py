# Copyright 2020 Giovani Bernardes
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Gazebo with a world that has Scooby, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_scooby_gazebo = get_package_share_directory('scooby_gazebo')
    scooby_description_pkg=get_package_share_directory('scooby_description')
    #scooby_node_pkg=get_package_share_directory('motors_node')
    
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    urdf_file= LaunchConfiguration('urdf_file')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        #condition=(IfCondition(LaunchConfiguration('verbose')))
        #condition=IfCondition(LaunchConfiguration('init')),
        #condition=IfCondition(LaunchConfiguration('factory')),
        #condition=IfCondition(LaunchConfiguration('force_system'))
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("joint_states", "scooby/joint_states")
        ],
        arguments=[urdf_file])

    start_scooby_teleop_cmd = Node(
        condition=IfCondition(use_teleop),
        package='scooby_teleop',
        executable='scooby_teleop',
        name='scooby_teleop',
        #output='screen',
        #remappings=[
        #    ("/scooby/cmd_vel", "/cmd_vel")
        #],
        #parameters=[{'use_sim_time': False}],
        )

    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        arguments=['-d', os.path.join(scooby_description_pkg, 'rviz', 'scooby_default.rviz')],
        condition=IfCondition(use_rviz)
    )

    controller = Node(
        package='calibration_controllers',
        executable='linear_controller',
        name='linear_controller',
        parameters=[{'use_sim_time': True}],
        output='screen')
        
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_scooby_gazebo, 'worlds', 'scooby_empty.world'), ''],
          description='SDF world file'),
        
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Set verbose to "true" to run gzserver.'),
        DeclareLaunchArgument('init', default_value='false',
                              description='Set init to "false" to run gzserver.'),
        DeclareLaunchArgument('factory', default_value='false',
                              description='Set factory to "false" to run gzserver.'),
        DeclareLaunchArgument('force_system', default_value='false',
                              description='Set force_system to "false" to run gzserver.'),
                   
                              
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Open scooby_teleop'),
        DeclareLaunchArgument('urdf_file',default_value=os.path.join(scooby_description_pkg, 'urdf', 'scooby.urdf'),
                              description='urddeclare_urdf_cmd =f file complete path'),
        gazebo,        
        robot_state_publisher,
        #start_scooby_teleop_cmd,
        controller
        #rviz
    ])
