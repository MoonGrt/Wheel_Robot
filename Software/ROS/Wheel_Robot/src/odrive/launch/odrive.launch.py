from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 ODrive 里程计节点
    odrive_node = Node(
        package='odrive',  # 替换为实际的 ROS 2 包名
        executable='odrive',  # 替换为实际的可执行文件名
        name='odrive_node',
        output='screen',
        parameters=[{
            'port': '/dev/odrive_uart',
            'baudrate': '921600',
        }],
    )

    # 添加静态 TF 变换 (odom 在 base_link 正上方 10cm)
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom',
        arguments=['0', '0', '0.1',  # x, y, z -> (0, 0, 0.1)
                   '0', '0', '0', '1',  # 四元数 (qx, qy, qz, qw)
                   'base_link', 'odom'],  # 父坐标系 base_link，子坐标系 odom
    )

    return LaunchDescription([
        odrive_node,
        tf_static_node,
    ])
