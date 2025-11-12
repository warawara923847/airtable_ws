#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include "wifi_config.hpp" // Wi-Fi設定

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3_stamped.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
Servo servo3;

rcl_publisher_t pub_gyro, pub_accel, pub_servo, pub_euler, pub_theta;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Vector3Stamped gyro_msg;
geometry_msgs__msg__Vector3Stamped accel_msg;
geometry_msgs__msg__Vector3Stamped euler_msg; // heading(yaw), roll, pitch in radians
std_msgs__msg__Float32 servo_msg;
std_msgs__msg__Float32 angle_msg; // 手動設定姿勢角 [deg]
std_msgs__msg__Float32 theta_msg; // 姿勢角のみpublish用

bool pid_enabled = true; // PID制御有効フラグ
float manual_angle = 90.0; // 手動設定姿勢角 [deg]
rcl_subscription_t sub;
rcl_subscription_t sub_angle;
std_msgs__msg__Bool recv_msg;
rclc_executor_t executor;

rcl_subscription_t sub_imu_reset;
std_msgs__msg__Bool imu_reset_msg;

float deadband = 0.1; // デッドバンドの閾値
float servo_center = 90.0; // 初期速度0
//const float gain = 10.0; // サーボ速度ゲイン
const int centerPulse = 1500; // サーボ中立パルス幅 [us]
int minUs = 500;
int maxUs = 2500;

float theta = 0.0f; //姿勢角 [rad]
float omega = 0.0f; //角速度 [rad/s]
float int_theta = 0.0f; //姿勢角の積分 [rad]
float prev_raw_theta = 0.0f; // 前回のセンサー生値 [rad]
float unwrap_offset = 0.0f;  // 累積オフセット [rad]

float Kp = 0.2; // P制御ゲイン
float Kd = 0.03; // D制御ゲイン
float Ki = 0.005; // I制御ゲイン
float dt = 0.04; // 制御周期 [s]
float gain = 30000; // PWMスケーリング

unsigned long lastTime = 0;
unsigned int fail_count = 0; // IMU再起動用

float unwrap_angle(float raw_angle) {
  // センサー生値との差分を計算
  float diff = raw_angle - prev_raw_theta;
  
  // -π〜π の範囲に正規化
  while (diff > PI) diff -= 2.0 * PI;
  while (diff < -PI) diff += 2.0 * PI;
  
  // 累積オフセットを更新
  unwrap_offset += diff;
  
  // 前回値を更新
  prev_raw_theta = raw_angle;
  
  return unwrap_offset;
}

void toggle_cb(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  pid_enabled = msg->data;
  Serial.printf("PID %s\n", pid_enabled ? "ENABLED" : "DISABLED");
}

void angle_cb(const void *msgin){
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  manual_angle = msg->data;
  Serial.printf("Manual angle: %.1f deg\n", manual_angle);
}

void imu_reset_cb(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    Serial.println("[ESP] IMU reset command received!");
    bno.begin();
    bno.setExtCrystalUse(true);
    
    // PID制御関連の状態をリセット
    int_theta = 0.0f;
    theta = 0.0f;
    prev_raw_theta = 0.0f;
    unwrap_offset = 0.0f;
    Serial.println("[ESP] PID states reset");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  
  // ==== Wi-Fi 接続 ====
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  // micro-ROS エージェントに接続
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);

  while(rmw_uros_ping_agent(1000, 5) != RMW_RET_OK){
      Serial.println("Waiting for micro-ROS agent...");
      delay(2000);
  }

  Serial.println("Connected to micro-ROS agent!");

  // micro-ROS初期化
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rmw_uros_sync_session(1000);
  rclc_node_init_default(&node, "esp32_wifi_node", "/", &support);

  rclc_publisher_init_default(
    &pub_gyro, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
    "imu/gyro");

  rclc_publisher_init_default(
    &pub_accel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
    "imu/accel");
  
  rclc_publisher_init_default(
    &pub_servo, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo/cmd");

  rclc_publisher_init_default(
    &pub_euler, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
    "imu/euler");
  
  rclc_publisher_init_default(
    &pub_theta, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "imu/theta"
  );

  rclc_subscription_init_default(
    &sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "pid_toggle");

  rclc_subscription_init_default(
    &sub_angle, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "manual_angle");
  
  rclc_subscription_init_default(
    &sub_imu_reset, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "imu_reset"
  );

  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &sub, &recv_msg, &toggle_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_angle, &angle_msg, &angle_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_imu_reset, &imu_reset_msg, &imu_reset_cb, ON_NEW_DATA);

  // IMU初期化
  if (!bno.begin()) {
    Serial.println("No BNO055 detected");
    while (1);
  }
  bno.setExtCrystalUse(true);

  // サーボ初期化
  servo1.attach(0, minUs, maxUs);
  servo2.attach(15, minUs, maxUs);
  servo1.write(servo_center);

  Serial.println("Setup complete");
}

void loop() {
  // 定期的に時刻同期（10秒ごと）
  static unsigned long last_sync_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_sync_time > 10000) {
    rmw_uros_sync_session(1000);
    last_sync_time = current_time;
  }

  // Agent の ping チェックを1秒ごとに変更（毎回だと遅い）
  static unsigned long last_ping_time = 0;
  if (current_time - last_ping_time > 1000) {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {  // タイムアウトを500ms→100msに短縮
      fail_count++;
      if (fail_count > 5) {  // 5回連続失敗で再起動
          Serial.println("Agent unreachable, restarting ESP...");
          delay(100);
          servo2.write(servo_center); // サーボ停止
          ESP.restart();
      }
    } else {
      fail_count = 0;
    }
    last_ping_time = current_time;
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  // 10ms→1msに短縮
  
  // ========================================
  // IMU データ取得とpublish（PID有効/無効に関わらず常に実行）
  // ========================================
  static unsigned long last_imu_pub_time = 0;
  unsigned long current_loop_time = millis();
  
  // IMU publishを適切な頻度に（40msごと = 25Hz、WiFi負荷を考慮）
  if (current_loop_time - last_imu_pub_time >= 40) {
    unsigned long pub_start = millis();
    unsigned long interval = pub_start - last_imu_pub_time;
    
    unsigned long t1 = millis();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    unsigned long t2 = millis();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    unsigned long t3 = millis();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // heading, roll, pitch (deg)
    unsigned long t4 = millis();
    
    if (last_imu_pub_time > 0 && interval > 45) {
      Serial.printf("[WARN] IMU interval: %lums (read:%lums)\n", 
                    interval, t4-t1);
    }
    last_imu_pub_time = current_loop_time;
    
    // 統一されたタイムスタンプを取得（1回のみ）
    int64_t now_nanos = rmw_uros_epoch_nanos();
    int32_t stamp_sec = (int32_t)(now_nanos / 1000000000LL);
    uint32_t stamp_nanosec = (uint32_t)(now_nanos % 1000000000LL);
    const float deg2rad = 0.017453292519943295f;
    
    // データ設定（publishの前にまとめて設定）
    gyro_msg.header.stamp.sec = stamp_sec;
    gyro_msg.header.stamp.nanosec = stamp_nanosec;
    gyro_msg.header.frame_id.data = (char *)"base_link";
    gyro_msg.vector.x = gyro.x();
    gyro_msg.vector.y = gyro.y();
    gyro_msg.vector.z = gyro.z();
    
    accel_msg.header.stamp.sec = stamp_sec;
    accel_msg.header.stamp.nanosec = stamp_nanosec;
    accel_msg.header.frame_id.data = (char *)"base_link";
    accel_msg.vector.x = accel.x();
    accel_msg.vector.y = accel.y();
    accel_msg.vector.z = accel.z();
    
    euler_msg.header.stamp.sec = stamp_sec;
    euler_msg.header.stamp.nanosec = stamp_nanosec;
    euler_msg.header.frame_id.data = (char *)"base_link";
    euler_msg.vector.x = euler.x() * deg2rad;
    euler_msg.vector.y = euler.y() * deg2rad;
    euler_msg.vector.z = euler.z() * deg2rad;
    
    // 一括publish（エラーチェックなし、高速化）
    rcl_publish(&pub_gyro, &gyro_msg, NULL);
    rcl_publish(&pub_accel, &accel_msg, NULL);
    rcl_publish(&pub_euler, &euler_msg, NULL);
    
    // PID制御が有効な場合のみ、theta/omegaを更新
    if (pid_enabled) {
      float raw_theta = euler.x() * deg2rad; // 姿勢角 [rad]
      theta = unwrap_angle(raw_theta);
      omega = gyro.z() * deg2rad; // 角速度 [rad/s]
    }
  }
  
  // ========================================
  // PID制御（自動モード）または手動モード
  // ========================================
  if (pid_enabled) {
    unsigned long now = millis();
    if (now - lastTime >= (unsigned long)(dt*1000.0f + 0.5f)){
      lastTime = now;

      // ==== PID制御 ====
      int_theta += theta * dt; // 積分項
      float Tc = - Kp * theta + Kd * omega + Ki * int_theta; // 制御量

      // ==== PWM司令に変換 ====
      int pulse = centerPulse + (int)(gain * Tc);
      pulse = constrain(pulse, minUs, maxUs);
      servo2.writeMicroseconds(pulse);
      servo_msg.data = (float)pulse;
      theta_msg.data = theta;
      rcl_publish(&pub_servo, &servo_msg, NULL);
      rcl_publish(&pub_theta, &theta_msg, NULL);
      Serial.printf("Theta: %.2f deg, Omega: %.2f deg/s, Pulse: %d us\n", theta*57.2958, omega*57.2958, pulse);
    } 
  } else {
    // 手動モード
    servo2.write(manual_angle);
  }
}

