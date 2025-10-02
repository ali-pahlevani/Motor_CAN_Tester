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
        'robot_real_move.urdf.xacro'
    )
    processed_urdf = xacro.process_file(urdf_xacro_path).toxml()
    robot_description = {"robot_description": processed_urdf}

    robot_control_config_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'can_move_controller.yaml'
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

    tri_cycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tri_cycle_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"publish_frequency": 30.0}],
    )

    delayed_controllers = TimerAction(
        period=5.0,  # seconds after launch
        actions=[control_node, tri_cycle_controller_spawner, joint_state_broadcaster_spawner]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        delayed_controllers,
    ])