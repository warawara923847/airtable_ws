from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    "/home/fujiitsu/ros2_ws/src/localization_pkg/config/ekf.yaml"
                ],
                arguments=["--ros-args", "--log-level", "debug"],
            )
        ]
    )
