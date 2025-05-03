import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # Robot
    package_name = 'wheel_robot'
    urdf_name = "wheel_robot.urdf"
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # 机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )


    # Motor
    motor_node = Node(
        package='motor',
        executable='motor',
        name='motor_node',
        output='screen',
    )


    # IMU
    imu_node = Node(
        package='imu',
        executable='imu',
        name='imu_node',
        output='screen',
    )


    # YDlidar
    share_dir = get_package_share_directory('ydlidar')
    parameter_file = LaunchConfiguration('params_file')
    ydlidar_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    ydlidar_node = LifecycleNode(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )


    # 使用 Node 启动 rosbridge_websocket
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket_node',
        output='screen',
    )


    # 启动 HTTP 服务器
    web_ctrl_dir = os.path.expanduser('../web_ctrl')
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        output='screen',
        cwd=web_ctrl_dir
    )

    return LaunchDescription([
        robot_state_publisher_node,
        motor_node,
        imu_node,
        ydlidar_params,
        ydlidar_node,
        rosbridge_websocket_node,
        # http_server, # TODO: Finish
    ])
