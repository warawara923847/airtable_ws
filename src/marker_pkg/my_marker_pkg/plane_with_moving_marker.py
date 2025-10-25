#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    Point,
    TwistStamped,
    Pose2D,
    PoseWithCovarianceStamped,
)
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import numpy as np
import tf_transformations
from collections import deque


class PlaneMarkerNode(Node):
    def __init__(self):
        super().__init__("plane_with_moving_marker")
        self.tf_braodcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = TransformBroadcaster(self)

        # パラメータ
        self.fixed_ids = (
            self.declare_parameter("fixed_marker_ids", [10, 11, 12, 13])
            .get_parameter_value()
            .integer_array_value
        )
        self.moving_id = (
            self.declare_parameter("moving_marker_id", 50)
            .get_parameter_value()
            .integer_value
        )
        self.offset_height = (
            self.declare_parameter("offset_height", 0.1)
            .get_parameter_value()
            .double_value
        )

        # 固定マーカーの位置を保持
        self.fixed_poses = {}
        self.initialized = False  # 初期化フラグ

        # 平滑化用バッファ
        self.normal_history = deque(maxlen=10)
        self.centroid_history = deque(maxlen=10)

        # サブスクライブ（固定マーカー）
        for mid in self.fixed_ids:
            topic = f"/aruco_single_{mid}/pose"
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, mid=mid: self.fixed_callback(msg, mid),
                10,
            )

        # サブスクライブ（可動マーカー）
        self.create_subscription(
            PoseStamped,
            f"/aruco_single_{self.moving_id}/pose",
            self.moving_callback,
            10,
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/moving_marker_pose", 10
        )
        self.rect_pub = self.create_publisher(Marker, "/plane_rectangle", 10)

        # 位置姿勢/速度角速度のPublish用
        self.pose2d_pub = self.create_publisher(Pose2D, "moving_marker_pose2d", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "moving_marker_velocity", 10)

        self.last_time = None
        self.last_pose = None
        self.last_quat = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 初期化状態管理 ---
        self.initialized = False
        self.base_initialized = False
        self.initial_centroid = None
        self.initial_q_plane = None

        # --- 各種pose記録 ---
        self.fixed_poses = {}
        self.moving_marker_pose = None

        self.get_logger().info("PlaneMarkerNode started")

    def initialize_plane_frame(self):
        pts = np.array(list(self.fixed_poses.values()))
        centroid = np.mean(pts, axis=0)
        v1 = pts[1] - pts[0]
        v2 = pts[2] - pts[0]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)

        z_axis = normal
        x_axis = pts[1] - pts[0]
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        R = np.vstack([x_axis, y_axis, z_axis]).T

        M = np.eye(4)
        M[:3, :3] = R
        q_plane = tf_transformations.quaternion_from_matrix(M)

        self.initial_R = R
        self.initial_centroid = centroid
        self.initial_q_plane = q_plane
        self.initialized = True

        self.get_logger().info("plane_frame initialized")
        return True

    def publish_plane_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = "plane_frame"
        t.transform.translation.x = self.initial_centroid[0]
        t.transform.translation.y = self.initial_centroid[1]
        t.transform.translation.z = self.initial_centroid[2]
        t.transform.rotation.x = self.initial_q_plane[0]
        t.transform.rotation.y = self.initial_q_plane[1]
        t.transform.rotation.z = self.initial_q_plane[2]
        t.transform.rotation.w = self.initial_q_plane[3]
        self.tf_broadcaster.sendTransform(t)

    def fixed_callback(self, msg, mid):
        # マーカーが見えた時にだけ更新
        self.fixed_poses[mid] = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

        # 初期化チェック（最初に4つ揃ったら初期化完了）
        if not self.initialized and len(self.fixed_poses) == len(self.fixed_ids):
            self.get_logger().info(
                "All fixed markers detected, plane_frame initialized"
            )
            success = self.initialize_plane_frame()
            if success:
                self.initialized = True
            else:
                self.get_logger().warn("Failed to initialize plane_frame")

    def publish_ideal_rectangle(self, frame_id="plane_frame", length=2.0, width=1.0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "plane"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # 線の太さ
        marker.scale.x = 0.002

        # 線の色（青）
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # 頂点（plane_frame 基準）
        half_l = length / 2.0
        half_w = width / 2.0
        corners = [
            (half_l, half_w, 0.0),
            (-half_l, half_w, 0.0),
            (-half_l, -half_w, 0.0),
            (half_l, -half_w, 0.0),
        ]

        for c in corners:
            pt = Point()
            pt.x, pt.y, pt.z = c
            marker.points.append(pt)
        # 枠を閉じる
        marker.points.append(marker.points[0])

        self.rect_pub.publish(marker)

    # --- base_link初期化 ---
    def initialize_base_link(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "plane_frame"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.static_tf_broadcaster.sendTransform(t)

        self.get_logger().info("Initialized base_link at moving marker pose origin.")

    def moving_callback(self, msg):
        if not self.initialized:
            self.get_logger().warn("Waiting for all fixed markers (initialization)")
            return

        # 初回受信時にbase_link初期化
        # if not self.base_initialized:
        #     self.initialize_base_link(msg)
        #     self.base_initialized = True

        self.publish_plane_tf()

        # --- 可動マーカー処理 ---
        pm = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        relative = pm - self.initial_centroid
        local = self.initial_R.T @ relative

        # Zを高さオフセットに置換
        local[2] = self.offset_height

        # --- Orientation: convert incoming marker quaternion (camera frame)
        # into the plane_frame quaternion and publish full quaternion.
        q_cam = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        # Convert to rotation matrix, rotate into plane_frame and convert back
        # to quaternion. initial_R maps plane_frame -> camera frame, so we
        # pre-multiply by its transpose to go camera -> plane.
        M_cam = tf_transformations.quaternion_matrix(q_cam)
        M_plane = np.eye(4)
        M_plane[:3, :3] = self.initial_R.T @ M_cam[:3, :3]
        corrected_q = tf_transformations.quaternion_from_matrix(M_plane)

        # --- Pose 出力 ---
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "plane_frame"
        pose_msg.pose.pose.position.x = local[0]
        pose_msg.pose.pose.position.y = local[1]
        pose_msg.pose.pose.position.z = local[2]
        pose_msg.pose.pose.orientation.x = corrected_q[0]
        pose_msg.pose.pose.orientation.y = corrected_q[1]
        pose_msg.pose.pose.orientation.z = corrected_q[2]
        pose_msg.pose.pose.orientation.w = corrected_q[3]

        pose_msg.pose.covariance = [
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.001,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            10.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.02,
        ]

        self.pose_pub.publish(pose_msg)

    # --- Pose2D出力 ---
    # extract yaw from the plane-frame quaternion for compatibility
    _, _, yaw = tf_transformations.euler_from_quaternion(corrected_q)
    pose2d = Pose2D()
    pose2d.x = local[0]
    pose2d.y = local[1]
    pose2d.theta = yaw
    self.pose2d_pub.publish(pose2d)

    # --- 速度出力 ---
        now = self.get_clock().now()
        if self.last_time is not None and self.last_pose2d is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 1e-3:
                dx = pose2d.x - self.last_pose2d.x
                dy = pose2d.y - self.last_pose2d.y
                dyaw = yaw - self.last_pose2d.theta
                dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

        vx = dx / dt
        vy = dy / dt
        wz = dyaw / dt

                twist = TwistStamped()
                twist.header.stamp = now.to_msg()
                twist.header.frame_id = "plane_frame"
                twist.twist.linear.x = vx
                twist.twist.linear.y = vy

                # Compute 3D angular velocity from quaternion difference
                if self.last_quat is not None:
                    q_prev = self.last_quat
                    q_curr = corrected_q
                    # q_delta = q_curr * conj(q_prev)
                    q_prev_conj = tf_transformations.quaternion_conjugate(q_prev)
                    q_delta = tf_transformations.quaternion_multiply(q_curr, q_prev_conj)

                    # Ensure numerical stability
                    qdw = np.clip(q_delta[3], -1.0, 1.0)
                    angle = 2.0 * np.arccos(qdw)
                    s = np.sqrt(1.0 - qdw * qdw)
                    if s < 1e-6:
                        # For very small angles, approximate axis by vector part
                        axis = np.array([q_delta[0], q_delta[1], q_delta[2]])
                    else:
                        axis = np.array([q_delta[0], q_delta[1], q_delta[2]]) / s

                    angular_vel = axis * (angle / dt)
                    twist.twist.angular.x = float(angular_vel[0])
                    twist.twist.angular.y = float(angular_vel[1])
                    twist.twist.angular.z = float(angular_vel[2])
                else:
                    twist.twist.angular.x = 0.0
                    twist.twist.angular.y = 0.0
                    twist.twist.angular.z = wz

                self.vel_pub.publish(twist)

        # store lasts for next velocity computation
        self.last_time = now
        self.last_pose2d = pose2d
        # quaternion in plane_frame
        self.last_quat = corrected_q

        # --- 可動マーカーのTFも出力 ---
        t2 = TransformStamped()
        t2.header = pose_msg.header
        t2.child_frame_id = "moving_marker_frame"
        t2.transform.translation.x = pose_msg.pose.pose.position.x
        t2.transform.translation.y = pose_msg.pose.pose.position.y
        t2.transform.translation.z = pose_msg.pose.pose.position.z
        t2.transform.rotation = pose_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t2)

        # --- 枠出力 ---
        self.publish_ideal_rectangle(length=0.5, width=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlaneMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
