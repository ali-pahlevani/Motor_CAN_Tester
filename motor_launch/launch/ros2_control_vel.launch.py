import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    urdf_xacro_path = os.path.join(
        get_package_share_directory('motor_description'),
        'urdf',
        'robot_real_vel.urdf.xacro'
    )
    processed_urdf = xacro.process_file(urdf_xacro_path).toxml()
    robot_description = {"robot_description": processed_urdf}

    robot_control_config_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'can_vel_controller.yaml'
    )

    slave_launch_path = os.path.join(
        get_package_share_directory('motor_launch'),
        'launch',
        'can_control_launch',
        'slave_launcher.launch.py'
    )

    slave_1_config_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'robot_control',
        'Kinco_FD_20250417_V1.2.eds'
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    traction_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["traction_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch_path),
        launch_arguments={
            "node_id": "2",
            "node_name": "traction_motor",
            "slave_config": slave_1_config_path,
        }.items(),
    )

    delayed_controllers = TimerAction(
        period=0.0,  # seconds after launch
        actions=[control_node, traction_velocity_controller_spawner, joint_state_broadcaster_spawner]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        #slave_node_1,
        delayed_controllers,
    ])