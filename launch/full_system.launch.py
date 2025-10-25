from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    use_camera = LaunchConfiguration('use_camera').perform(context)
    use_marker = LaunchConfiguration('use_marker').perform(context)

    rs_pkg = get_package_share_directory('realsense2_camera')
    joy_pkg = get_package_share_directory('joy_esp_bridge')
    imu_pkg = get_package_share_directory('imu_alignment_pkg')
    marker_pkg = get_package_share_directory('my_marker_pkg')
    loc_pkg = get_package_share_directory('localization_pkg')

    actions = []

    # --- RealSense camera ---
    if use_camera == 'true':
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(rs_pkg, 'launch', 'rs_launch.py')
            ))
        )

    # --- ESP32 joystick bridge ---
    actions.append(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(joy_pkg, 'launch', 'joy_esp_system.launch.py')
        ))
    )

    # --- IMU alignment node (plane_frame → base_link TF発行) ---
    imu_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(imu_pkg, 'launch', 'imu_align.launch.py')
    ))
    actions.append(imu_launch)

    # --- ArUco marker detection system ---
    if use_marker == 'true':
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(marker_pkg, 'launch', 'marker_system.launch.py')
            ))
        )

    # --- EKF node (robot_localization) ---
    ekf_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(loc_pkg, 'launch', 'ekf_launch.py')
    ))

    # ✅ EKFを IMUノード起動後に数秒遅らせて起動
    delayed_ekf = TimerAction(
        period=5.0,  # IMU/ArUco初期化を待つ
        actions=[ekf_launch]
    )
    actions.append(delayed_ekf)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true', description='Launch realsense node'),
        DeclareLaunchArgument('use_marker', default_value='true', description='Launch ArUco marker node'),
        OpaqueFunction(function=launch_setup)
    ])

