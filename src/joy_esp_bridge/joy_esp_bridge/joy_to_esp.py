#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
import time


class JoyToEsp(Node):
    def __init__(self):
        super().__init__("joy_to_esp")
        self.sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.pub_toggle = self.create_publisher(Bool, "pid_toggle", 10)
        self.pub_angle = self.create_publisher(Float32, "manual_angle", 10)
        self.pub_imu_reset = self.create_publisher(Bool, "imu_reset", 10)

        # 内部状態
        self.pid_mode = False
        self.extended_mode = False

        self.normal_angles = [80, 85, 90, 95, 100]  # 通常モードの角度リスト
        self.extended_angles = [0, 90, 180]  # 拡張モードの角度リスト

        self.angle_list = self.normal_angles
        self.angle_idx = self.angle_list.index(90)  # 初期角度90度

        self.dpad_axis = 5  # 十字キーのY軸（D-pad Up/Down）

        # デバウンス用
        self.last_toggle_time = 0.0
        self.debounce_interval = 0.3  # 秒

        # 状態保持（ボタン・軸の前回値）
        self.prev_buttons = [0] * 12
        self.prev_axes = [0.0] * 8

    def joy_callback(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        now = time.time()

        # === AボタンでPIDトグル ===
        if len(buttons) > 0:
            if (
                buttons[1] == 1
                and (now - self.last_toggle_time) > self.debounce_interval
            ):
                self.pid_mode = not self.pid_mode
                self.pub_toggle.publish(Bool(data=self.pid_mode))
                self.last_toggle_time = now
                self.get_logger().info(f"[PID MODE] {'ON' if self.pid_mode else 'OFF'}")

        if (
            len(buttons) > 2
            and buttons[2] == 1
            and (now - self.last_toggle_time) > self.debounce_interval
        ):
            self.pub_imu_reset.publish(Bool(data=True))
            self.last_toggle_time = now
            self.get_logger().info("[IMU] Reset command sent")

        # === 十字キー（D-pad）で角度制御 ===
        if not self.pid_mode:  # ← PIDモードOFF時のみ角度変更を許可！
            # Bボタンで角度範囲切替
            if (
                len(buttons) > 1
                and buttons[0] == 1
                and (now - self.last_toggle_time) > self.debounce_interval
            ):
                self.extended_mode = not self.extended_mode
                self.last_toggle_time = now

                self.angle_list = (
                    self.extended_angles if self.extended_mode else self.normal_angles
                )
                self.angle_idx = self.angle_list.index(90)
                self.pub_angle.publish(Float32(data=90.0))

                mode_name = (
                    "EXTENDED (0/90/180)" if self.extended_mode else "NORMAL (80-100)"
                )
                self.get_logger().info(f"[ANGLE MODE] {mode_name}, reset to 90°")

            if len(axes) > self.dpad_axis:
                dpad_y = axes[self.dpad_axis]
                if dpad_y == 1.0 and self.prev_axes[self.dpad_axis] != 1.0:
                    self.increment_angle(+1)
                elif dpad_y == -1.0 and self.prev_axes[self.dpad_axis] != -1.0:
                    self.increment_angle(-1)
            else:
                # PIDモード中にD-padを押しても反応しない（デバッグ出力のみ）
                if len(axes) > self.dpad_axis:
                    dpad_y = axes[self.dpad_axis]
                if abs(dpad_y) > 0.5:
                    self.get_logger().info("Ignored D-pad input (PID mode active)")

            # 状態更新
            self.prev_buttons = list(buttons)
            self.prev_axes = list(axes)

    def increment_angle(self, direction):
        new_idx = max(0, min(len(self.angle_list) - 1, self.angle_idx + direction))
        if new_idx != self.angle_idx:
            self.angle_idx = new_idx
            new_angle = self.angle_list[self.angle_idx]
            self.pub_angle.publish(Float32(data=float(new_angle)))
            mode = "EXT" if self.extended_mode else "NORM"
            self.get_logger().info(f"Angle switched to {new_angle:.1f}° ({mode})")


def main(args=None):
    rclpy.init(args=args)
    node = JoyToEsp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
