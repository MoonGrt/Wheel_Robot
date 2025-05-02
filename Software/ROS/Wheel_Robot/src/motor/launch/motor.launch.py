from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 motor 里程计节点
    motor_node = Node(
        package='motor',
        executable='motor',
        name='motor_node',
        output='screen',
    )

    return LaunchDescription([
        motor_node,
    ])
