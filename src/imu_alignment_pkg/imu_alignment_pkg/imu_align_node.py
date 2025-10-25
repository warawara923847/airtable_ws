import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Vector3Stamped,
    QuaternionStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
)
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

        # Data storage
        self.gyro = None
        self.accel = None
        self.euler = None
        self.aruco_quat = None
        self.offset_rot = None

        # Parameters
        self.calibration_done = False

        self.get_logger().info("IMU Alignment Node (Yaw Calibration Ver.) started ✅")

    # --- Callbacks -----------------------------------------------------

    def gyro_callback(self, msg: Vector3Stamped):
        self.gyro = msg.vector
        self.try_publish()

    def accel_callback(self, msg: Vector3Stamped):
        self.accel = msg.vector
        self.try_publish()

    def euler_callback(self, msg: Vector3Stamped):
        self.euler = msg.vector
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

        # 一度だけキャリブレーション実行
        if not self.calibration_done and self.euler is not None:
            imu_rot = R.from_euler(
                "zyx", [self.euler.x, self.euler.z, self.euler.y]
            )  # yaw, pitch, roll
            self.offset_rot = self.aruco_quat * imu_rot.inv()
            self.calibration_done = True
            self.get_logger().info(
                "✅ IMU Yaw alignment calibrated using ArUco orientation"
            )

    # --- Main publishing logic ----------------------------------------

    def try_publish(self):
        # 必要な3種類のデータが揃っていない場合
        if self.gyro is None or self.accel is None or self.euler is None:
            return

        imu_rot = R.from_euler(
            "zyx", [self.euler.x, self.euler.z, self.euler.y]
        )  # yaw, pitch, roll

        # オフセットが設定されている場合 → 姿勢補正
        if self.offset_rot is not None:
            imu_rot = self.offset_rot * imu_rot

        quat = imu_rot.as_quat()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "plane_frame"

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

        if not self.calibration_done:
            # Humble以降では info_throttle が削除されたため、自前で5秒おきに出力
            now = self.get_clock().now().nanoseconds
            if not hasattr(self, "_last_log_time"):
                self._last_log_time = 0
            if now - self._last_log_time > 5 * 1e9:  # 5秒おき
                self.get_logger().info(
                    "Waiting for ArUco orientation for yaw calibration..."
                )
                self._last_log_time = now


# ---------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = ImuAlignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
