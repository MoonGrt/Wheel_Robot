import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_name = 'wheel_robot'
    urdf_name = "wheel_robot.urdf"
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
        )

    # Odom
    odom_node = Node(
        package='odom',
        executable='odom',
        name='odom_node',
        output='screen',
        parameters=[{
            'port': '/dev/odrive_uart',
            'baudrate': '921600',
        }],
    )
    odom_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom',
        arguments=['0', '0', '0',  # x, y, z -> (0, 0, 0)
                   '0', '0', '0', '1',  # 四元数 (qx, qy, qz, qw)
                   'base_link', 'odom'],  # 父坐标系 base_link，子坐标系 odom
    )

    # IMU
    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu_node',
        output='screen',
        parameters=[
            {'port': '/dev/imu'},
            {'baudrate': '921600'}
        ]
    )
    imu_tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=['0', '0', '0.2',  # 位置 (x, y, z) -> (0, 0, 0.2)
                    '0', '0', '0', '1',  # 旋转 (四元数 qx, qy, qz, qw) -> (0, 0, 0, 1)
                    'base_link', 'imu']  # 父坐标系 'base_link'，子坐标系 'imu'
    )

    # YDlidar
    share_dir = get_package_share_directory('ydlidar')
    parameter_file = LaunchConfiguration('params_file')
    ydlidar_params = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'ydlidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    ydlidar_node = LifecycleNode(package='ydlidar',
                                node_executable='ydlidar_node',
                                node_name='ydlidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )
    ydlidar_tf2_node = Node(
        package='tf2_ros',  # 指定使用 tf2_ros 包
        node_executable='static_transform_publisher',  # 运行 static_transform_publisher 可执行文件
        node_name='static_tf_pub_laser',  # 该节点的名称
        arguments=['0', '0', '0.04',  # 位置 (x, y, z) -> (0, 0, 0.04)
                '0', '0', '0', '1',  # 旋转 (四元数 qx, qy, qz, qw) -> (0, 0, 0, 1) (即无旋转)
                'base_link', 'laser'],  # 父坐标系 'base_link'，子坐标系 'laser'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        odom_node,
        odom_tf2_node,
        imu_node,
        imu_tf2_node,
        ydlidar_params,
        ydlidar_node,
        ydlidar_tf2_node,
    ])
