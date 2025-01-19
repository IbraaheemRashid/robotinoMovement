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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Copy test files to share directory
        (os.path.join('lib', package_name, 'test'), glob('test/*')),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'requests',
    ],
    zip_safe=True,
    maintainer='mirashid',
    maintainer_email='mirashid@todo.todo',
    description='Hardware Abstraction Layer for Robotino Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = robotino_hal.nodes.battery_monitor:main',
            'sensor_monitor = robotino_hal.nodes.sensor_monitor:main',
            'motor_control = robotino_hal.nodes.motor_control:main',
            'odometry_monitor = robotino_hal.nodes.odometry_monitor:main',
        ],
    },
)