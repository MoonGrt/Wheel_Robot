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
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
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


    return LaunchDescription([
        robot_state_publisher_node,
        odom_node,
        imu_node,
        ydlidar_params,
        ydlidar_node,
    ])
