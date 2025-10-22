from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # --- joy.yaml の絶対パス取得 ---
    joy_config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'joy_esp_bridge',
        'joy.yaml'
    )
    joy_config = os.path.abspath(joy_config)

    return LaunchDescription([

        # ジョイノード
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=['joy_config'] # joy.yaml のパスを指定
        ),

        # joy → ESPトピックブリッジ
        Node(
            package='joy_esp_bridge',
            executable='joy_to_esp',
            name='joy_to_esp',
            output='screen'
        ),

        # micro-ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', '--port', '8888']
        ),
    ])

