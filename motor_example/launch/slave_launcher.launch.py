import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
      """Generate launch description with multiple components."""
      path_file = os.path.dirname(__file__)

      ld = launch.LaunchDescription()


      device_container = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
              [
                  os.path.join(get_package_share_directory("canopen_core"), "launch"),
                  "/canopen.launch.py",
              ]
          ),
          launch_arguments={
              "master_config": os.path.join(
                  get_package_share_directory("motor_example"),
                  "config",
                  "robot_control",
                  "master.dcf",
              ),
              #"master_bin": os.path.join(
              #    get_package_share_directory("motor_example"),
              #    "config",
              #    "robot_control",
              #    "master.bin",
              #),
              "bus_config": os.path.join(
                  get_package_share_directory("motor_example"),
                  "config",
                  "robot_control",
                  "bus.yml",
              ),
              "can_interface_name": "can0",
          }.items(),

      )

      ld.add_action(device_container)

      return ld