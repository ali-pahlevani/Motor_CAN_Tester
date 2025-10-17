import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    urdf_xacro_path = os.path.join(
        get_package_share_directory('motor_description'),
        'urdf',
        'robot_real_all.urdf.xacro'
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('motor_launch'),
        'rviz',
        'robot_view.rviz'
    )

    # Process xacro
    processed_urdf = xacro.process_file(urdf_xacro_path).toxml()

    # Launch argument for GUI
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Use joint state publisher GUI'
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': processed_urdf
        }]
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
