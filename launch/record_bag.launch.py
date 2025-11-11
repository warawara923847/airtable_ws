from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime
import yaml
import subprocess
import atexit
import shlex

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
            '/aruco_single_10/pose',
            '/aruco_single_11/pose',
            '/aruco_single_12/pose',
            '/aruco_single_13/pose',
            '/aruco_single_50/pose',
            '/aruco_single_10/marker',
            '/aruco_single_11/marker',
            '/aruco_single_12/marker',
            '/aruco_single_13/marker',
            '/aruco_single_50/marker',
            # '/aruco_single_50/result',
            '/imu/aligned',
            '/imu/euler',
            '/imu/theta',
            '/moving_marker_pose',
            '/moving_marker_pose2d',
            '/moving_marker_velocity',
            '/plane_rectangle', # ArUco検出平面の四隅(rviz用)
            '/odometry/filtered',
            '/servo/cmd',
            '/tf', '/tf_static'
        ]

        # === 終了時の後処理: summary.yaml 保存 + 再圧縮を非同期起動 ===
        def _on_shutdown(_context):
            # 1) summary.yaml を bag_dir に保存
            try:
                if os.path.isdir(bag_dir):
                    info_cmd = ['ros2', 'bag', 'info', bag_dir]
                    info_output = subprocess.check_output(info_cmd).decode('utf-8', errors='ignore')
                    summary_path = os.path.join(bag_dir, 'summary.yaml')
                    with open(summary_path, 'w') as f:
                        f.write(info_output)
                    print(f"[record_bag.launch.py] Summary saved to {summary_path}")
                else:
                    print(f"[record_bag.launch.py] Bag directory not found. Skipping summary.")
            except Exception as e:
                # ros2 bag info が失敗したら最低限のメタデータだけ出す
                try:
                    summary_path = os.path.join(bag_dir, 'summary.yaml')
                    with open(summary_path, 'w') as f:
                        yaml.dump({'experiment': exp_name, 'timestamp': timestamp}, f)
                    print(f"[record_bag.launch.py] Fallback summary saved to {summary_path} (reason: {e})")
                except Exception as e2:
                    print(f"[record_bag.launch.py] Failed to write fallback summary: {e2}")

            # 2) 再圧縮（変換）スクリプトを低優先度でバックグラウンド起動
            #    scripts/convert_one_bag.sh <bag_dir> が存在すれば起動
            script = os.path.join(ws_dir, 'scripts', 'convert_one_bag.sh')
            if os.path.exists(script) and os.path.isdir(bag_dir):
                cmdline = f'nice -n 10 ionice -c2 -n7 {shlex.quote(script)} {shlex.quote(bag_dir)}'
                try:
                    subprocess.Popen(cmdline, cwd=ws_dir, shell=True)
                    print(f"[record_bag.launch.py] Spawned converter: {cmdline}")
                except Exception as e:
                    print(f"[record_bag.launch.py] Failed to spawn converter: {e}")
            else:
                print(f"[record_bag.launch.py] Converter script not found or bag_dir missing. Skip.")

            return []

        actions = [
            ExecuteProcess(cmd=cmd, shell=False, output='screen'),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_on_shutdown)])),
        ]
        print('[INFO] recording to:', bag_dir)
        return actions

    return LaunchDescription([
        exp_name_arg,
        OpaqueFunction(function=launch_setup)
    ])

