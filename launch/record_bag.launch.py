from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime
import yaml
import subprocess
import atexit

def generate_launch_description():

    exp_name_arg = DeclareLaunchArgument(
        'exp_name',
        default_value='default_experiment',
        description='Experiment name for rosbag folder'
    )

    def launch_setup(context, *args, **kwargs):
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        data_dir = os.path.join(ws_dir, 'data', 'bags')
        os.makedirs(data_dir, exist_ok=True)

        exp_name = LaunchConfiguration('exp_name').perform(context)
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bag_dir = os.path.join(data_dir, f'{timestamp}_{exp_name}')

        cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-a', '--include-hidden-topics'
        ]

        # Register cleanup to write summary.yaml when launch ends
        def cleanup():
            if not os.path.exists(bag_dir):
                print(f"[record_bag.launch.py] Bag directory not found. Skipping summary.")
                return
            info_cmd = ['ros2', 'bag', 'info', bag_dir]
            summary_path = os.path.join(bag_dir, 'summary.yaml')
            try:
                info_output = subprocess.check_output(info_cmd).decode('utf-8')
                with open(summary_path, 'w') as f:
                    f.write(info_output)
            except Exception as e:
                print(f"[record_bag.launch.py] Failed to get bag info: {e}")
                with open(summary_path, 'w') as f:
                    yaml.dump({'experiment': exp_name, 'timestamp': timestamp}, f)
            print(f"[record_bag.launch.py] Summary saved to {summary_path}")

        atexit.register(cleanup)

        # Use ExecuteProcess so launch manages rosbag’s lifecycle
        return [ExecuteProcess(cmd=cmd, output='screen')]

    return LaunchDescription([
        exp_name_arg,
        OpaqueFunction(function=launch_setup)
    ])

