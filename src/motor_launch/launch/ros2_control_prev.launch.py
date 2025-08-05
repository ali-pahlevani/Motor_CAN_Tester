from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("motor_description"),
                    "urdf",
                    "robot_real.urdf.xacro",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("motor_control"), "config", "can_controller.yaml"]
    )
  
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_control_config],
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

    slave_1_config = PathJoinSubstitution(
        [FindPackageShare("motor_control"), "config/robot_control", "Kinco_FD_20250417_V1.2.eds"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("motor_launch"), "launch/can_control_launch", "slave_launcher.launch.py"]
    )
    
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "2",
            "node_name": "traction_motor",
            "slave_config": slave_1_config,
        }.items(),
    )
    
    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        traction_velocity_controller_spawner,
        robot_state_publisher_node,
        slave_node_1,
    ]

    return LaunchDescription(nodes_to_start)