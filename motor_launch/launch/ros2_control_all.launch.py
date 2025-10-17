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
        'robot_real_all.urdf.xacro'
    )
    processed_urdf = xacro.process_file(urdf_xacro_path).toxml()
    robot_description = {"robot_description": processed_urdf}

    robot_control_config_path = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'can_all_controller.yaml'
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

    forks_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forks_position_controller", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],#, {"publish_frequency": 30.0}],
    )

    delayed_controllers = TimerAction(
        period=5.0,  # seconds after launch
        actions=[control_node, tri_cycle_controller_spawner, forks_position_controller_spawner, joint_state_broadcaster_spawner]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[('/cmd_vel_in','/tri_cycle_controller/cmd_vel/unstamped'),
                    ('/cmd_vel_out','/tri_cycle_controller/cmd_vel')]
    )

    # Include pf_driver launch file
    pf_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pf_driver'),
                'launch',
                'r2000.launch.py'
            )
        )
    )

    # Include sick_scanner_driver launch file
    sick_scanner_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sick_safetyscanners2'),
                'launch',
                'sick_safetyscanners2_launch.py'
                #'sick_safetyscanners2_lifecycle_launch.py'
            )
        )
    )

    # Static transform publisher for lidar_nav_link
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            #"--x", "0.037",
            #"--y", "0.0",
            #"--z", "0.75",
            #"--qx", "0.0",
            #"--qy", "0.0",
            #"--qz", "0.0",
            #"--qw", "1.0",
            "--x", "0.19",
            "--y", "-0.30",
            "--z", "0.01",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "base_link",
            "--child-frame-id", "lidar_nav_link"
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        delayed_controllers,
        #twist_stamper,
        pf_driver_launch,
        #sick_scanner_driver_launch,
        #static_transform_publisher
    ])