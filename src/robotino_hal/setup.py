from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotino_hal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
         (os.path.join('share', package_name, 'description'),
         glob('description/*')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mirashid',
    maintainer_email='mirashid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = nodes.battery_monitor:main',
            'sensor_monitor = nodes.sensor_monitor:main',
            'motor_control = nodes.motor_control:main',
            'odometry_monitor = nodes.odometry_monitor:main',
            'camera_monitor = nodes.camera_monitor:main',
            'navigation = nodes.navigation:main',
            'lidar_monitor = nodes.lidar_monitor:main',
        ],
    }
)