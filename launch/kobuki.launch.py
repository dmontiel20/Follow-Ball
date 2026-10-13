# Copyright 2024 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('follow_ball')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # camera_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('follow_ball'),
    #         'launch',
    #         'hsv.filter.launch.py')),
    #     )

    # laser_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('follow_ball'),
    #         'launch',
    #         'obstacle.detector.launch.py')),
    #     )

    follow_ball_cmd = Node(
        package='follow_ball',
        executable='control_node_main',
        name='follow_ball',
        output='screen',
        parameters=[param_file],
        remappings=[('velocity', '/cmd_vel')])

    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    ld = LaunchDescription()

    # ld.add_action(camera_cmd)
    # ld.add_action(laser_cmd)
    ld.add_action(follow_ball_cmd)
    ld.add_action(rviz2_cmd)

    return ld
