from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_alignment_pkg',
            executable='imu_align_node',
            name='imu_align_node',
            output='screen'
        )
    ])

