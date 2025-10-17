from setuptools import find_packages, setup

package_name = 'motor_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/can_control_launch', ['launch/can_control_launch/slave_launcher.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_control_vel.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_control_pos.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_control_move.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_control_all.launch.py']),
        ('share/' + package_name + '/launch/etc', ['launch/etc/urdf_test_vel.launch.py']),
        ('share/' + package_name + '/launch/etc', ['launch/etc/urdf_test_pos.launch.py']),
        ('share/' + package_name + '/launch/etc', ['launch/etc/urdf_test_move.launch.py']),
        ('share/' + package_name + '/launch/etc', ['launch/etc/urdf_test_all.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/robot_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='Launch files and configurations for autofork',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
