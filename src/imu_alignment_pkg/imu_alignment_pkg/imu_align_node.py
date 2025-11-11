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
        
        # Re-publish用（タイムスタンプ修正済み）
        self.pub_euler_corrected = self.create_publisher(Vector3Stamped, "/imu/euler_corrected", 10)
        self.pub_gyro_corrected = self.create_publisher(Vector3Stamped, "/imu/gyro_corrected", 10)
        self.pub_accel_corrected = self.create_publisher(Vector3Stamped, "/imu/accel_corrected", 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Data storage
        self.gyro = None
        self.accel = None
        self.euler = None  # Vector3Stamped メッセージ全体を保存
        self.aruco_quat = None
        self.offset_rot = None
        self.last_aruco_quat = None  # 診断用：最新のArUco姿勢を保持
        
        # 停止検出用
        self.last_publish_time = 0
        self.was_stopped = False
        self.euler_at_stop = None

        # Parameters
        self.calibration_done = False
        self._last_diagnostic_time = 0
        
        # デバッグ：コールバック呼び出しカウンタ
        self._euler_callback_count = 0
        self._last_euler_value = None
        self._last_callback_report_time = 0
        self._last_data_status_report_time = 0  # データ到着状況のレポート用
        self._euler_total_callbacks = 0  # 起動からの総コールバック数
        self._publish_count = 0  # /imu/aligned の publish 回数

        self.get_logger().info("IMU Alignment Node started ✅")
        self.get_logger().info("Re-publishing with corrected timestamps:")
        self.get_logger().info("  /imu/euler → /imu/euler_corrected")
        self.get_logger().info("  /imu/gyro → /imu/gyro_corrected")
        self.get_logger().info("  /imu/accel → /imu/accel_corrected")

    # --- Callbacks -----------------------------------------------------

    def gyro_callback(self, msg: Vector3Stamped):
        # ESP32のタイムスタンプは不正確なので現在時刻で上書き
        from geometry_msgs.msg import Vector3Stamped as V3S
        corrected_msg = V3S()
        corrected_msg.header.stamp = self.get_clock().now().to_msg()
        corrected_msg.header.frame_id = msg.header.frame_id
        corrected_msg.vector = msg.vector
        self.gyro = corrected_msg
        
        # 修正されたタイムスタンプで re-publish
        self.pub_gyro_corrected.publish(corrected_msg)
        
        self.try_publish()

    def accel_callback(self, msg: Vector3Stamped):
        # ESP32のタイムスタンプは不正確なので現在時刻で上書き
        from geometry_msgs.msg import Vector3Stamped as V3S
        corrected_msg = V3S()
        corrected_msg.header.stamp = self.get_clock().now().to_msg()
        corrected_msg.header.frame_id = msg.header.frame_id
        corrected_msg.vector = msg.vector
        self.accel = corrected_msg
        
        # 修正されたタイムスタンプで re-publish
        self.pub_accel_corrected.publish(corrected_msg)
        
        self.try_publish()

    def euler_callback(self, msg: Vector3Stamped):
        self._euler_total_callbacks += 1
        
        # 角度を -π ~ π の範囲に正規化してメッセージ全体を保存
        from geometry_msgs.msg import Vector3Stamped as V3S
        normalized_msg = V3S()
        normalized_msg.header.stamp = self.get_clock().now().to_msg()  # 現在時刻で上書き
        normalized_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "base_link"
        normalized_msg.vector.x = np.arctan2(np.sin(msg.vector.x), np.cos(msg.vector.x))
        normalized_msg.vector.y = np.arctan2(np.sin(msg.vector.y), np.cos(msg.vector.y))
        normalized_msg.vector.z = np.arctan2(np.sin(msg.vector.z), np.cos(msg.vector.z))
        self.euler = normalized_msg
        
        # 修正されたタイムスタンプで re-publish
        self.pub_euler_corrected.publish(normalized_msg)
        
        # デバッグ：euler値が実際に変化しているか確認（5秒おき）
        self._euler_callback_count += 1
        now = self.get_clock().now().nanoseconds
        if now - self._last_callback_report_time > 5 * 1e9:
            euler_changed = "CHANGED" if self._last_euler_value is None or \
                (abs(self.euler.vector.x - self._last_euler_value.x) > 0.01 or \
                 abs(self.euler.vector.y - self._last_euler_value.y) > 0.01 or \
                 abs(self.euler.vector.z - self._last_euler_value.z) > 0.01) else "STATIC"
            self.get_logger().info(
                f"📊 IMU euler callbacks: {self._euler_callback_count} in last ~5s, "
                f"current=(x:{self.euler.vector.x:.3f}, y:{self.euler.vector.y:.3f}, z:{self.euler.vector.z:.3f}), "
                f"status={euler_changed}"
            )
            from geometry_msgs.msg import Vector3
            self._last_euler_value = Vector3(x=self.euler.vector.x, y=self.euler.vector.y, z=self.euler.vector.z)
            self._euler_callback_count = 0
            self._last_callback_report_time = now
        
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
        self.last_aruco_quat = self.aruco_quat  # 診断用に保存

        # 一度だけキャリブレーション実行
        if not self.calibration_done and self.euler is not None:
            # 起動直後の不安定なデータをスキップ（最初の10メッセージ）
            # if self._euler_total_callbacks < 10:
            #     self.get_logger().warn(
            #         f"⚠️ IMU initializing ({self._euler_total_callbacks}/10 messages), skipping calibration"
            #     )
            #     return

            # ゼロ値チェック：IMU が初期化中の場合はスキップ
            if abs(self.euler.vector.x) < 0.001 and abs(self.euler.vector.y) < 0.001 and abs(self.euler.vector.z) < 0.001:
                self.get_logger().warn(
                    "⚠️ IMU euler is zero (sensor initializing), skipping calibration"
                )
                return
            
            # ESP32 の /imu/euler の軸定義:
            #   euler.x = yaw (Z軸周り回転)
            #   euler.y = roll (X軸周り回転)
            #   euler.z = pitch (Y軸周り回転)
            # ※ ROS標準(x=roll, y=pitch, z=yaw)とは異なる順序
            imu_rot = R.from_euler(
                "zyx", [self.euler.vector.x, self.euler.vector.z, self.euler.vector.y]
            )  # yaw, pitch, roll の順
            self.offset_rot = self.aruco_quat * imu_rot.inv()
            self.calibration_done = True
            
            # 詳細なキャリブレーション情報を出力
            aruco_euler = self.aruco_quat.as_euler('zyx')  # yaw, pitch, roll
            imu_euler = imu_rot.as_euler('zyx')
            self.get_logger().info(
                f"✅ IMU alignment calibrated:\n"
                f"  ArUco orientation (yaw, pitch, roll): "
                f"({np.rad2deg(aruco_euler[0]):.1f}°, {np.rad2deg(aruco_euler[1]):.1f}°, {np.rad2deg(aruco_euler[2]):.1f}°)\n"
                f"  IMU orientation (yaw, pitch, roll): "
                f"({np.rad2deg(imu_euler[0]):.1f}°, {np.rad2deg(imu_euler[1]):.1f}°, {np.rad2deg(imu_euler[2]):.1f}°)\n"
                f"  Raw IMU euler: (x={self.euler.vector.x:.3f}, y={self.euler.vector.y:.3f}, z={self.euler.vector.z:.3f})\n"
                f"  Offset will align IMU to ArUco reference"
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
        # データ到着状況を定期的にレポート（5秒おき）
        now = self.get_clock().now().nanoseconds
        if now - self._last_data_status_report_time > 5 * 1e9:
            status = f"gyro={'OK' if self.gyro else 'MISSING'}, " \
                     f"accel={'OK' if self.accel else 'MISSING'}, " \
                     f"euler={'OK' if self.euler else 'MISSING'}"
            if self.gyro is None or self.accel is None or self.euler is None:
                self.get_logger().warn(
                    f"⚠️ Waiting for IMU data: {status} "
                    f"(/imu/aligned NOT publishing)"
                )
            else:
                # タイムスタンプのずれをチェック
                current_time = self.get_clock().now().nanoseconds
                euler_age_ms = (current_time - self.euler.header.stamp.sec * 1e9 - self.euler.header.stamp.nanosec) / 1e6
                gyro_age_ms = (current_time - self.gyro.header.stamp.sec * 1e9 - self.gyro.header.stamp.nanosec) / 1e6
                accel_age_ms = (current_time - self.accel.header.stamp.sec * 1e9 - self.accel.header.stamp.nanosec) / 1e6
                self.get_logger().info(
                    f"⏱️ IMU data age: euler={euler_age_ms:.0f}ms, gyro={gyro_age_ms:.0f}ms, accel={accel_age_ms:.0f}ms"
                )
            self._last_data_status_report_time = now
        
        # 必要な3種類のデータが揃っていない場合
        if self.gyro is None or self.accel is None or self.euler is None:
            # デバッグ：どのデータが欠けているか毎回ログ出力
            if self.calibration_done:
                euler_val = f"(x={self.euler.vector.x:.3f}, y={self.euler.vector.y:.3f}, z={self.euler.vector.z:.3f})" if self.euler else "N/A"
                self.get_logger().warn(
                    f"⚠️ try_publish() early return: gyro={'OK' if self.gyro else 'MISSING'}, "
                    f"accel={'OK' if self.accel else 'MISSING'}, euler={'OK' if self.euler else 'MISSING'}\n"
                    f"  Last euler value: {euler_val}",
                    throttle_duration_sec=1.0
                )
            return
        
        # データの鮮度チェック（3つのタイムスタンプの差が1秒以上ある場合は古いデータとみなす）
        euler_ts = self.euler.header.stamp.sec + self.euler.header.stamp.nanosec * 1e-9
        gyro_ts = self.gyro.header.stamp.sec + self.gyro.header.stamp.nanosec * 1e-9
        accel_ts = self.accel.header.stamp.sec + self.accel.header.stamp.nanosec * 1e-9
        
        max_ts = max(euler_ts, gyro_ts, accel_ts)
        min_ts = min(euler_ts, gyro_ts, accel_ts)
        time_diff = max_ts - min_ts
        
        if time_diff > 1.0:  # 1秒以上の差がある場合
            self.get_logger().warn(
                f"⚠️ IMU data timestamps differ by {time_diff:.2f}s - data too old, skipping publish\n"
                f"  euler={euler_ts:.3f}s, gyro={gyro_ts:.3f}s, accel={accel_ts:.3f}s",
                throttle_duration_sec=2.0
            )
            # 古いデータをクリアして、次回は新しいデータを待つ
            if euler_ts < gyro_ts - 0.5:
                self.euler = None
                self.get_logger().info("Cleared old euler data")
            if gyro_ts < euler_ts - 0.5:
                self.gyro = None
                self.get_logger().info("Cleared old gyro data")
            if accel_ts < euler_ts - 0.5:
                self.accel = None
                self.get_logger().info("Cleared old accel data")
            return

        # ESP32 の /imu/euler の軸定義に合わせたマッピング
        # euler.x=yaw, euler.y=roll, euler.z=pitch
        imu_rot = R.from_euler(
            "zyx", [self.euler.vector.x, self.euler.vector.z, self.euler.vector.y]
        )  # yaw, pitch, roll の順

        # オフセットが設定されている場合 → 姿勢補正
        if self.offset_rot is not None:
            imu_rot = self.offset_rot * imu_rot

        quat = imu_rot.as_quat()

        imu_msg = Imu()
        # 元のIMUデータのタイムスタンプを使用（3つの中で最新のもの）
        euler_ts = self.euler.header.stamp.sec + self.euler.header.stamp.nanosec * 1e-9
        gyro_ts = self.gyro.header.stamp.sec + self.gyro.header.stamp.nanosec * 1e-9
        accel_ts = self.accel.header.stamp.sec + self.accel.header.stamp.nanosec * 1e-9
        
        # 最新のタイムスタンプを持つメッセージを選択
        if euler_ts >= gyro_ts and euler_ts >= accel_ts:
            imu_msg.header.stamp = self.euler.header.stamp
        elif gyro_ts >= accel_ts:
            imu_msg.header.stamp = self.gyro.header.stamp
        else:
            imu_msg.header.stamp = self.accel.header.stamp
            
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

        imu_msg.angular_velocity.x = self.gyro.vector.x
        imu_msg.angular_velocity.y = self.gyro.vector.y
        imu_msg.angular_velocity.z = self.gyro.vector.z
        imu_msg.angular_velocity_covariance = [
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

        imu_msg.linear_acceleration.x = self.accel.vector.x
        imu_msg.linear_acceleration.y = self.accel.vector.y
        imu_msg.linear_acceleration.z = self.accel.vector.z
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
        self._publish_count += 1
        
        # 停止から再開を検出
        current_time = self.get_clock().now().nanoseconds
        if self.last_publish_time > 0 and (current_time - self.last_publish_time) > 2e9:  # 2秒以上空いた
            if not self.was_stopped:
                self.was_stopped = True
                self.euler_at_stop = (self.euler.vector.x, self.euler.vector.y, self.euler.vector.z)
                self.get_logger().warn(
                    f"⚠️ /imu/aligned publishing STOPPED for >2s\n"
                    f"  euler at stop: (x={self.euler.vector.x:.3f}, y={self.euler.vector.y:.3f}, z={self.euler.vector.z:.3f})"
                )
            else:
                # 再開
                if self.euler_at_stop:
                    dx = self.euler.vector.x - self.euler_at_stop[0]
                    dy = self.euler.vector.y - self.euler_at_stop[1]
                    dz = self.euler.vector.z - self.euler_at_stop[2]
                    self.get_logger().warn(
                        f"🔄 /imu/aligned publishing RESUMED\n"
                        f"  euler now: (x={self.euler.vector.x:.3f}, y={self.euler.vector.y:.3f}, z={self.euler.vector.z:.3f})\n"
                        f"  euler change during stop: (Δx={dx:.3f}, Δy={dy:.3f}, Δz={dz:.3f})\n"
                        f"  This change will cause alignment drift!"
                    )
                self.was_stopped = False
                self.euler_at_stop = None
        else:
            self.was_stopped = False
        
        self.last_publish_time = current_time
        
        # 最初の数回は publish 成功をログ出力
        if self._publish_count <= 5 or self._publish_count % 50 == 0:
            # タイムスタンプの差を確認（既に計算済み）
            aligned_ts = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            
            self.get_logger().info(
                f"📤 Published /imu/aligned (count={self._publish_count})\n"
                f"  Timestamps: euler={euler_ts:.3f}s, gyro={gyro_ts:.3f}s, accel={accel_ts:.3f}s, aligned={aligned_ts:.3f}s",
                throttle_duration_sec=1.0
            )
        
        # キャリブレーション後の定期診断（3秒おき）
        if self.calibration_done and self.last_aruco_quat is not None:
            now = self.get_clock().now().nanoseconds
            if now - self._last_diagnostic_time > 3 * 1e9:
                # 現在の補正済みIMU姿勢と最新のArUco姿勢を比較
                aligned_euler = imu_rot.as_euler('zyx')  # yaw, pitch, roll
                aruco_euler = self.last_aruco_quat.as_euler('zyx')
                yaw_diff = np.rad2deg(aruco_euler[0] - aligned_euler[0])
                
                self.get_logger().info(
                    f"🔍 Alignment check:\n"
                    f"  ArUco yaw={np.rad2deg(aruco_euler[0]):.1f}°, "
                    f"Aligned IMU yaw={np.rad2deg(aligned_euler[0]):.1f}°, "
                    f"diff={yaw_diff:.1f}°\n"
                    f"  Raw IMU euler: (x={self.euler.vector.x:.3f}, y={self.euler.vector.y:.3f}, z={self.euler.vector.z:.3f})"
                )
                self._last_diagnostic_time = now


# ---------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ImuAlignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
