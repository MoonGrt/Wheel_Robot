import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'wheel_robot'
    package_name = 'wheel_robot'
    urdf_name = "wheel_robot.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_urdf_model_path = os.path.join(pkg_share, f'urdf/gazebo_{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/simulation.world')

    # 读取 URDF 文件并替换 package:// 路径为绝对路径
    with open(urdf_model_path, "r") as file:
        urdf_content = file.read()
    urdf_content = urdf_content.replace("package://wheel_robot", pkg_share)
    # 写入替换后的 URDF 文件
    with open(gazebo_urdf_model_path, "w", encoding="utf-8") as file:
        file.write(urdf_content)

    # Start Gazebo server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-file', gazebo_urdf_model_path ], output='screen')

    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[gazebo_urdf_model_path]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path]
        )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_rviz_cmd)

    return ld
