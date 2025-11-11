from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # マーカーのサイズ [m]
    marker_size = 0.1  # 例: 10cm

    # 基準平面の4隅のマーカーID
    plane_ids = [10, 11, 12, 13]
    # 可動マーカーのID
    moving_id = 50

    nodes = []

    # --- カメラトピック設定 ---
    IR_IMAGE = "/camera/camera/infra1/image_rect_raw"
    IR_INFO = "/camera/camera/infra1/camera_info"
    IR_FRAME = "camera_infra1_optical_frame"

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
                        "camera_frame": IR_FRAME,
                        "reference_frame": IR_FRAME,
                        "marker_frame": f"aruco_marker_{mid}",
                    }
                ],
                remappings=[
                    ("/camera_info", IR_INFO),
                    ("/image", IR_IMAGE),
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
                    "camera_frame": IR_FRAME,
                    "reference_frame": IR_FRAME,
                    "marker_frame": f"aruco_marker_{moving_id}",
                }
            ],
            remappings=[
                ("/camera_info", IR_INFO),
                ("/image", IR_IMAGE),
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
                {"height_offset": 0.147},  # 基準平面からの高さ[m]
                # {"height_offset": 0.0}, # デバッグ用
            ],
        )
    )

    return LaunchDescription(nodes)
