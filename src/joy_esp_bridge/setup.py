from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joy_esp_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['joy_esp_bridge/joy.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fujiitsu',
    maintainer_email='fujiitsu@todo.todo',
    description='Bridge joystick input to ESP32 servo control via micro-ROS',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_to_esp = joy_esp_bridge.joy_to_esp:main',
        ],
    },
)
