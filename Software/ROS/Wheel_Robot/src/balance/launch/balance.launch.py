from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu_node',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
    ])
