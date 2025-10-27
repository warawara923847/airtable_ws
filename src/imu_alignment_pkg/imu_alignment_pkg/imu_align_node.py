import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Vector3Stamped,
    QuaternionStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from tf2_ros import TransformBroadcaster
import tf_transformations
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np


class ImuAlignNode(Node):
    def __init__(self):
        super().__init__("imu_align_node")

        # Subscribers
        self.create_subscription(Vector3Stamped, "/imu/gyro", self.gyro_callback, 10)
        self.create_subscription(Vector3Stamped, "/imu/accel", self.accel_callback, 10)
        self.create_subscription(Vector3Stamped, "/imu/euler", self.euler_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/moving_marker_pose", self.aruco_callback, 10
        )

        # Publisher
        self.pub = self.create_publisher(Imu, "/imu/aligned", 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Data storage
        self.gyro = None
        self.accel = None
        self.euler = None
        self.euler_timestamp = None  # eulerデータの鮮度確認用
        self.aruco_quat = None
        self.offset_rot = None
        self.last_aruco_msg = None  # 診断用に最新のArUcoメッセージを保持

        # Parameters
        self.calibration_done = False
        self._last_log_time = 0
        self._last_diagnostic_time = 0
        self.euler_is_degrees = None  # IMU Eulerが度かラジアンか自動検出

        self.get_logger().info("IMU Alignment Node (Yaw Calibration Ver.) started ✅")

    # --- Callbacks -----------------------------------------------------

    def gyro_callback(self, msg: Vector3Stamped):
        self.gyro = msg.vector
        self.try_publish()

    def accel_callback(self, msg: Vector3Stamped):
        self.accel = msg.vector
        self.try_publish()

    def euler_callback(self, msg: Vector3Stamped):
        # IMU Eulerが度かラジアンか自動検出（初回のみ）
        if self.euler_is_degrees is None:
            max_val = max(abs(msg.vector.x), abs(msg.vector.y), abs(msg.vector.z))
            if max_val > 3.15:  # πより大きければ度と判定
                self.euler_is_degrees = True
                self.get_logger().warn(
                    f"⚠️ IMU Euler appears to be in DEGREES (max value: {max_val:.2f}). "
                    "Will convert to radians."
                )
            else:
                self.euler_is_degrees = False
                self.get_logger().info("✅ IMU Euler detected as RADIANS")
        
        # 度の場合はラジアンに変換
        if self.euler_is_degrees:
            self.euler = type(msg.vector)(
                x=np.deg2rad(msg.vector.x),
                y=np.deg2rad(msg.vector.y),
                z=np.deg2rad(msg.vector.z)
            )
        else:
            self.euler = msg.vector
        
        self.euler_timestamp = msg.header.stamp  # タイムスタンプを記録
        self.try_publish()

    def aruco_callback(self, msg: PoseWithCovarianceStamped):
        self.aruco_quat = R.from_quat(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.last_aruco_msg = msg  # 診断用に保存

        # 一度だけキャリブレーション実行
        if not self.calibration_done and self.euler is not None:
            # IMU Eulerがゼロの場合は警告（初期化されていない可能性）
            euler_magnitude = np.sqrt(self.euler.x**2 + self.euler.y**2 + self.euler.z**2)
            if euler_magnitude < 0.01:  # ほぼゼロ
                self.get_logger().warn(
                    f"⚠️ Deferring calibration: IMU euler is zero or near-zero "
                    f"({self.euler.x:.3f}, {self.euler.y:.3f}, {self.euler.z:.3f}). "
                    f"Waiting for valid IMU data..."
                )
                return
            
            # IMU Eulerデータの鮮度を確認（ArUcoより古いデータでキャリブレーションしない）
            if self.euler_timestamp is not None:
                aruco_stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                euler_stamp_ns = self.euler_timestamp.sec * 1e9 + self.euler_timestamp.nanosec
                age_diff = (aruco_stamp_ns - euler_stamp_ns) / 1e9
                
                # IMU Eulerが ArUco より 0.5秒以上古い場合はキャリブレーションを延期
                if age_diff > 0.5:
                    self.get_logger().warn(
                        f"⚠️ Deferring calibration: IMU euler is {age_diff:.2f}s older than ArUco. "
                        f"Waiting for fresh IMU data..."
                    )
                    return
            
            # Vector3: x=roll, y=pitch, z=yaw
            # from_euler("xyz") expects [roll, pitch, yaw]
            imu_rot = R.from_euler(
                "xyz", [self.euler.x, self.euler.y, self.euler.z]
            )
            self.offset_rot = self.aruco_quat * imu_rot.inv()
            self.calibration_done = True
            
            # 診断: キャリブレーション時の角度を出力
            aruco_euler = self.aruco_quat.as_euler('xyz')  # roll, pitch, yaw
            imu_euler_rad = [self.euler.x, self.euler.y, self.euler.z]
            self.get_logger().info(
                f"✅ IMU Yaw alignment calibrated:\n"
                f"  ArUco angles (r,p,y): ({aruco_euler[0]:.3f}, {aruco_euler[1]:.3f}, {aruco_euler[2]:.3f}) rad = "
                f"({np.rad2deg(aruco_euler[0]):.1f}, {np.rad2deg(aruco_euler[1]):.1f}, {np.rad2deg(aruco_euler[2]):.1f}) deg\n"
                f"  IMU angles (r,p,y):   ({imu_euler_rad[0]:.3f}, {imu_euler_rad[1]:.3f}, {imu_euler_rad[2]:.3f}) rad = "
                f"({np.rad2deg(imu_euler_rad[0]):.1f}, {np.rad2deg(imu_euler_rad[1]):.1f}, {np.rad2deg(imu_euler_rad[2]):.1f}) deg\n"
                f"  Yaw difference: {np.rad2deg(aruco_euler[2] - imu_euler_rad[2]):.1f} deg"
            )

            self.publish_plane_to_base_tf(msg)

    # --- TF Publishing -------------------------------------------------
    def publish_plane_to_base_tf(self, aruco_msg: PoseWithCovarianceStamped):
        """
        plane_frame → base_link のTFを一度だけ送信。
        ArUcoマーカー（plane_frame基準）とIMUの初期姿勢を同期させる。
        
        役割:
        - IMU キャリブレーション直後の初期 TF を出力
        - その後 EKF が動的に plane_frame → base_link を更新
        """
        if self.offset_rot is None:
            self.get_logger().warn("Offset rotation not computed yet, skipping TF.")
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "plane_frame"
        t.child_frame_id = "base_link"

        # ArUco から取得した位置を初期値として使用
        t.transform.translation.x = aruco_msg.pose.pose.position.x
        t.transform.translation.y = aruco_msg.pose.pose.position.y
        t.transform.translation.z = aruco_msg.pose.pose.position.z

        # キャリブレーションされた姿勢（offset_rot）を使用
        q = self.offset_rot.as_quat()  # x, y, z, w
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            "✅ Published initial plane_frame → base_link TF. "
            "EKF will now update it dynamically."
        )

    # --- Main publishing logic ----------------------------------------

    def try_publish(self):
        # 必要な3種類のデータが揃っていない場合
        if self.gyro is None or self.accel is None or self.euler is None:
            return

        # Vector3: x=roll, y=pitch, z=yaw
        # from_euler("xyz") expects [roll, pitch, yaw]
        imu_rot = R.from_euler(
            "xyz", [self.euler.x, self.euler.y, self.euler.z]
        )

        # オフセットが設定されている場合 → 姿勢補正
        if self.offset_rot is not None:
            imu_rot = self.offset_rot * imu_rot

        quat = imu_rot.as_quat()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        imu_msg.orientation_covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
        ]

        imu_msg.angular_velocity.x = self.gyro.x
        imu_msg.angular_velocity.y = self.gyro.y
        imu_msg.angular_velocity.z = self.gyro.z
        imu_msg.angular_velocity_covariance = [
            1e-4,
            0.0,
            0.0,
            0.0,
            1e-4,
            0.0,
            0.0,
            0.0,
            1e-4,
        ]

        imu_msg.linear_acceleration.x = self.accel.x
        imu_msg.linear_acceleration.y = self.accel.y
        imu_msg.linear_acceleration.z = self.accel.z
        imu_msg.linear_acceleration_covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
        ]

        # クォータニオンの正規化
        norm = np.linalg.norm(
            [
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w,
            ]
        )
        if norm > 0:
            imu_msg.orientation.x /= norm
            imu_msg.orientation.y /= norm
            imu_msg.orientation.z /= norm
            imu_msg.orientation.w /= norm

        self.pub.publish(imu_msg)

        # キャリブレーション前の待機メッセージ
        if not self.calibration_done:
            now = self.get_clock().now().nanoseconds
            if now - self._last_log_time > 5 * 1e9:  # 5秒おき
                self.get_logger().info(
                    "Waiting for ArUco orientation for yaw calibration..."
                )
                self._last_log_time = now
        
        # キャリブレーション後の定期診断（2秒おき）
        elif self.last_aruco_msg is not None and self.euler is not None:
            now = self.get_clock().now().nanoseconds
            if now - self._last_diagnostic_time > 2 * 1e9:
                # 現在の補正済みIMU yawと最新のArUco yawを比較
                aligned_euler = imu_rot.as_euler('xyz')
                aruco_euler = self.aruco_quat.as_euler('xyz')
                yaw_diff = np.rad2deg(aruco_euler[2] - aligned_euler[2])
                
                # タイムスタンプの鮮度を確認
                euler_age = "unknown"
                aruco_age = "unknown"
                if self.euler_timestamp is not None:
                    euler_age = f"{(now - (self.euler_timestamp.sec * 1e9 + self.euler_timestamp.nanosec)) / 1e9:.2f}s"
                if self.last_aruco_msg is not None:
                    aruco_stamp = self.last_aruco_msg.header.stamp
                    aruco_age = f"{(now - (aruco_stamp.sec * 1e9 + aruco_stamp.nanosec)) / 1e9:.2f}s"
                
                self.get_logger().info(
                    f"🔍 Diagnostic: ArUco yaw={np.rad2deg(aruco_euler[2]):.1f}° (age:{aruco_age}), "
                    f"Aligned IMU yaw={np.rad2deg(aligned_euler[2]):.1f}° (age:{euler_age}), "
                    f"diff={yaw_diff:.1f}°, "
                    f"raw IMU euler=({self.euler.x:.3f},{self.euler.y:.3f},{self.euler.z:.3f})"
                )
                self._last_diagnostic_time = now


# ---------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ImuAlignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
