from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 odom 里程计节点
    odrive_node = Node(
        package='odom',  # 替换为实际的 ROS 2 包名
        executable='odom',  # 替换为实际的可执行文件名
        name='odrive_node',
        output='screen',
        parameters=[{
            'port': '/dev/odrive_uart',
            'baudrate': '921600',
        }],
    )

    return LaunchDescription([
        odrive_node,
    ])
