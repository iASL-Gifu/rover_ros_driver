from setuptools import find_packages, setup

package_name = 'rover_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['rover_ros_driver', 'rover_ros_driver.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rover_ros_driver.launch.xml']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arata22',
    maintainer_email='ky.37f.9850@s.thers.ac.jp',
    description='ROS2 driver for CAN bus control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'can_sender_node=rover_ros_driver.can_sender_node:main',
        ],
    },
)
