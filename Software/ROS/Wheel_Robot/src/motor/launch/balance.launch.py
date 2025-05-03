from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # IMU
    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu_node',
        output='screen',
    )

    # Motor
    motor_node = Node(
        package='motor',
        executable='motor',
        name='motor_node',
        output='screen',
    )

    return LaunchDescription([
        imu_node,
        motor_node,
    ])
