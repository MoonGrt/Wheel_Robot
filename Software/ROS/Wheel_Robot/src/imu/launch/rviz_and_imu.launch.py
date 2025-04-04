from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_and_imu_node = Node(
        package='imu',
        node_executable='imu',
        node_name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/imu'},
                    {"baud": 9600}],
        output="screen"

    )

    rviz_display_node = Node(
        package='rviz2',
        node_executable="rviz2",
        output="screen"
    )

    return LaunchDescription(
        [
            rviz_and_imu_node,
            # rviz_display_node
        ]
    )