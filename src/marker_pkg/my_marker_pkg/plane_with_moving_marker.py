#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, TwistStamped, Pose2D
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import numpy as np
import tf_transformations
from collections import deque

class PlaneMarkerNode(Node):
    def __init__(self):
        super().__init__('plane_with_moving_marker')

        # パラメータ
        self.fixed_ids = self.declare_parameter('fixed_marker_ids', [10, 11, 12, 13]).get_parameter_value().integer_array_value
        self.moving_id = self.declare_parameter('moving_marker_id', 50).get_parameter_value().integer_value
        self.offset_height = self.declare_parameter('offset_height', 0.1).get_parameter_value().double_value

        # 固定マーカーの位置を保持
        self.fixed_poses = {}
        self.initialized = False  # 初期化フラグ

        # 平滑化用バッファ
        self.normal_history = deque(maxlen=10)
        self.centroid_history = deque(maxlen=10)

        # サブスクライブ（固定マーカー）
        for mid in self.fixed_ids:
            topic = f"/aruco_single_{mid}/pose"
            self.create_subscription(PoseStamped, topic, lambda msg, mid=mid: self.fixed_callback(msg, mid), 10)

        # サブスクライブ（可動マーカー）
        self.create_subscription(PoseStamped, f"/aruco_single_{self.moving_id}/pose", self.moving_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/moving_marker_pose", 10)
        self.rect_pub = self.create_publisher(Marker, "/plane_rectangle", 10)

        # 位置姿勢/速度角速度のPublish用
        self.pose2d_pub = self.create_publisher(Pose2D, "moving_marker_pose2d", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "moving_marker_velocity", 10)

        self.last_time = None
        self.last_pose = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("PlaneMarkerNode started")

    def fixed_callback(self, msg, mid):
        # マーカーが見えた時にだけ更新
        self.fixed_poses[mid] = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
            ])

        # 初期化チェック（最初に4つ揃ったら初期化完了）
        if not self.initialized and len(self.fixed_poses) == len(self.fixed_ids):
            self.initialized = True
            self.get_logger().info("All fixed markers detected, plane_frame initialized")

    def publish_ideal_rectangle(self, frame_id="plane_frame",
                                length=2.0, width=1.0):
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
                ( half_l,  half_w, 0.0),
                (-half_l,  half_w, 0.0),
                (-half_l, -half_w, 0.0),
                ( half_l, -half_w, 0.0)
                ]

        for c in corners:
            pt = Point()
            pt.x, pt.y, pt.z = c
            marker.points.append(pt)
        # 枠を閉じる
        marker.points.append(marker.points[0])

        self.rect_pub.publish(marker)


    def moving_callback(self, msg):
        if not self.initialized:
            self.get_logger().warn("Waiting for all fixed markers (initialization)")
            return

        # --- 平面計算（保持値も利用） ---
        pts = np.array(list(self.fixed_poses.values()))
        centroid = np.mean(pts, axis=0)
        v1 = pts[1] - pts[0]
        v2 = pts[2] - pts[0]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)

        # 移動平均で安定化
        self.normal_history.append(normal)
        self.centroid_history.append(centroid)

        normal = np.mean(self.normal_history, axis=0)
        normal /= np.linalg.norm(normal)
        centroid = np.mean(self.centroid_history, axis=0)

        # 座標系 (plane_frame)
        z_axis = normal
        x_axis = (pts[1] - pts[0])
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        R = np.vstack([x_axis, y_axis, z_axis]).T

        # --- plane_frame の TF 出力 ---
        M = np.eye(4)
        M[:3, :3] = R
        q_plane = tf_transformations.quaternion_from_matrix(M)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = "plane_frame"
        t.transform.translation.x = centroid[0]
        t.transform.translation.y = centroid[1]
        t.transform.translation.z = centroid[2]
        t.transform.rotation.x = q_plane[0]
        t.transform.rotation.y = q_plane[1]
        t.transform.rotation.z = q_plane[2]
        t.transform.rotation.w = q_plane[3]
        self.tf_broadcaster.sendTransform(t)

        # --- 可動マーカー処理 ---
        pm = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        relative = pm - centroid
        local = R.T @ relative

        # Zを高さオフセットに置換
        local[2] = self.offset_height

        # Yawのみ利用
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        corrected_q = tf_transformations.quaternion_from_euler(0, 0, yaw)

        # --- Pose 出力 ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "plane_frame"
        pose_msg.pose.position.x = local[0]
        pose_msg.pose.position.y = local[1]
        pose_msg.pose.position.z = local[2]
        pose_msg.pose.orientation.x = corrected_q[0]
        pose_msg.pose.orientation.y = corrected_q[1]
        pose_msg.pose.orientation.z = corrected_q[2]
        pose_msg.pose.orientation.w = corrected_q[3]

        self.pose_pub.publish(pose_msg)

        # --- Pose2D出力 ---
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
                dyaw = (dyaw + np.pi) % (2*np.pi) - np.pi

                vx = dx / dt
                vy = dy / dt
                wz = dyaw / dt

                twist = TwistStamped()
                twist.header.stamp = now.to_msg()
                twist.header.frame_id = "plane_frame"
                twist.twist.linear.x = vx
                twist.twist.linear.y = vy
                twist.twist.angular.z = wz
                self.vel_pub.publish(twist)

        self.last_time = now
        self.last_pose2d = pose2d


        # --- 可動マーカーのTFも出力 ---
        t2 = TransformStamped()
        t2.header = pose_msg.header
        t2.child_frame_id = "moving_marker_frame"
        t2.transform.translation.x = pose_msg.pose.position.x
        t2.transform.translation.y = pose_msg.pose.position.y
        t2.transform.translation.z = pose_msg.pose.position.z
        t2.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t2)

        # --- 枠出力 ---
        self.publish_ideal_rectangle(length=0.5, width=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlaneMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

