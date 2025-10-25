from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # マーカーのサイズ [m]
    marker_size = 0.075  # 例: 10cm

    # 基準平面の4隅のマーカーID
    plane_ids = [10, 11, 12, 13]
    # 可動マーカーのID
    moving_id = 50

    nodes = []

    # --- 基準マーカー4つ ---
    for mid in plane_ids:
        nodes.append(
            Node(
                package="aruco_ros",
                executable="single",
                name=f"aruco_single_{mid}",
                parameters=[
                    {
                        "marker_id": mid,
                        "marker_size": marker_size,
                        "camera_frame": "camera_color_optical_frame",
                        "reference_frame": "camera_color_optical_frame",
                        "marker_frame": f"aruco_marker_{mid}",
                    }
                ],
                remappings=[
                    ("/camera_info", "/camera/camera/color/camera_info"),
                    ("/image", "/camera/camera/color/image_raw"),
                ],
            )
        )

    # --- 可動マーカー ---
    nodes.append(
        Node(
            package="aruco_ros",
            executable="single",
            name=f"aruco_single_{moving_id}",
            parameters=[
                {
                    "marker_id": moving_id,
                    "marker_size": marker_size,
                    "camera_frame": "camera_color_optical_frame",
                    "reference_frame": "camera_color_optical_frame",
                    "marker_frame": f"aruco_marker_{moving_id}",
                }
            ],
            remappings=[
                ("/camera_info", "/camera/camera/color/camera_info"),
                ("/image", "/camera/camera/color/image_raw"),
            ],
        )
    )

    # --- 自作ノード (基準平面+可動マーカー処理) ---
    nodes.append(
        Node(
            package="my_marker_pkg",
            executable="plane_with_moving_marker",
            name="plane_with_moving_marker",
            parameters=[
                {"plane_marker_ids": plane_ids},
                {"moving_marker_id": moving_id},
                # {"height_offset": 0.157},  # 基準平面からの高さ[m]
                {"height_offset": 0.0}, # デバッグ用
            ],
        )
    )

    return LaunchDescription(nodes)
