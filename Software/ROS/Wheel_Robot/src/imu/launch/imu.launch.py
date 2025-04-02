from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu_node',
        output='screen',
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=['0', '0', '0.2',  # 位置 (x, y, z) -> (0, 0, 0.2)
                    '0', '0', '0', '1',  # 旋转 (四元数 qx, qy, qz, qw) -> (0, 0, 0, 1)
                    'base_link', 'imu']  # 父坐标系 'base_link'，子坐标系 'imu'
    )

    return LaunchDescription([
        imu_node,
        tf2_node,
    ])
