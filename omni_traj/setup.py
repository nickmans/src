from setuptools import setup
from glob import glob
import os

package_name = 'omni_traj'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nickolas',
    maintainer_email='nickolas@todo.todo',
    description='Waypoint -> costmap -> A* -> dt trajectory generator for omni robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'empty_scan_pub = omni_traj.empty_scan_pub:main',
            'lidar_watchdog = omni_traj.lidar_watchdog:main',
            'map_odom_startup_fallback = omni_traj.map_odom_startup_fallback:main',
            'waypoint_traj = omni_traj.waypoint_traj_node:main',
        ],
    },
)
