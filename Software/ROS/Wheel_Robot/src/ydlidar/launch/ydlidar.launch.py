#!/usr/bin/python3
# Copyright 2020, EAIBOT
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar')
    rviz_config_file = os.path.join(share_dir, 'config','ydlidar.rviz')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar',
                                node_executable='ydlidar_node',
                                node_name='ydlidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )

    tf2_node = Node(
        package='tf2_ros',  # 指定使用 tf2_ros 包
        node_executable='static_transform_publisher',  # 运行 static_transform_publisher 可执行文件
        node_name='static_tf_pub_laser',  # 该节点的名称
        arguments=['0', '0', '0.02',  # 位置 (x, y, z) -> (0, 0, 0.02)
                '0', '0', '0', '1',  # 旋转 (四元数 qx, qy, qz, qw) -> (0, 0, 0, 1) (即无旋转)
                'base_link', 'laser_link'],  # 父坐标系 'base_link'，子坐标系 'laser_link'
    )

    rviz2_node = Node(package='rviz2',
                    node_executable='rviz2',
                    node_name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        rviz2_node,
    ])
