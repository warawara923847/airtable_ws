#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include "wifi_config.hpp" // Wi-Fi設定

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
Servo servo3;

rcl_publisher_t pub_gyro, pub_accel, pub_servo, pub_euler;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

geometry_msgs__msg__Vector3 gyro_msg;
geometry_msgs__msg__Vector3 accel_msg;
geometry_msgs__msg__Vector3 euler_msg; // heading(yaw), roll, pitch in radians
std_msgs__msg__Float32 servo_msg;
std_msgs__msg__Float32 angle_msg; // 手動設定姿勢角 [deg]

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
float prev_theta = 0.0f;

float Kp = 0.05; // P制御ゲイン
float Kd = 0.02; // D制御ゲイン
float Ki = 0.001; // I制御ゲイン
float dt = 0.02; // 制御周期 [s]
float gain = 30000; // PWMスケーリング

unsigned long lastTime = 0;

float unwrap_angle(float prev, float current) {
  float diff = current - prev;
  if (diff > PI) {
    current -= 2.0 * PI;
  } else if (diff < -PI) {
    current += 2.0 * PI;
  }
  return current;
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
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  
  // ==== Wi-Fi 接続 ====
  // WiFi.begin(ssid, password);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  // micro-ROS エージェントに接続
  //set_microros_wifi_transports(ssid, password, agent_ip, AGENT_PORT);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  delay(2000);

  // micro-ROS初期化
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_wifi_node", "/", &support);

  rclc_publisher_init_default(
    &pub_gyro, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu/gyro");

  rclc_publisher_init_default(
    &pub_accel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu/accel");
  
  rclc_publisher_init_default(
    &pub_servo, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo/cmd");

  rclc_publisher_init_default(
    &pub_euler, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu/euler");

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
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (pid_enabled) {
    // put your main code here, to run repeatedly:
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // heading, roll, pitch (deg)
    rcl_ret_t ret;
    
    // IMU publish
    gyro_msg.x = gyro.x();
    gyro_msg.y = gyro.y();
    gyro_msg.z = gyro.z();
    ret = rcl_publish(&pub_gyro, &gyro_msg, NULL);
    if(ret != RCL_RET_OK) {
      Serial.printf("Error publishing gyro: %d\n", ret);
    }

    accel_msg.x = accel.x();
    accel_msg.y = accel.y();
    accel_msg.z = accel.z();
    ret = rcl_publish(&pub_accel, &accel_msg, NULL);
    if(ret != RCL_RET_OK) {
      Serial.printf("Error publishing accel: %d\n", ret);
    }

    // Euler angles publish (convert deg->rad)
    const float deg2rad = 0.017453292519943295f;
    euler_msg.x = euler.x() * deg2rad; // heading (yaw)
    euler_msg.y = euler.y() * deg2rad; // roll
    euler_msg.z = euler.z() * deg2rad; // pitch
    ret = rcl_publish(&pub_euler, &euler_msg, NULL);
    Serial.printf("Theta: %.2f deg", euler.x());
    if(ret != RCL_RET_OK) {
      Serial.printf("Error publishing euler: %d\n", ret);
    }

    unsigned long now = millis();
    if (now -lastTime >= (unsigned long)(dt*1000)){
      lastTime = now;

      float prev_theta = theta;
      float raw_theta = euler.z() * deg2rad; // 姿勢角 [rad]
      theta = unwrap_angle(prev_theta, raw_theta);

      omega = gyro.z() * deg2rad; // 角速度 [rad/s]

      // ==== PID制御 ====
      int_theta += theta * dt; // 積分項
      float Tc = Kp * theta + Kd * omega + Ki * int_theta; // 制御量

      // ==== PWM司令に変換 ====
      int pulse = centerPulse + (int)(gain * Tc);
      pulse = constrain(pulse, minUs, maxUs);
      servo2.writeMicroseconds(pulse);
      servo_msg.data = (float)pulse;
      ret = rcl_publish(&pub_servo, &servo_msg, NULL);
      Serial.printf("Theta: %.2f deg, Omega: %.2f deg/s, Pulse: %d us\n", theta*57.2958, omega*57.2958, pulse);
      if(ret != RCL_RET_OK) {
        Serial.printf("Error publishing servo: %d\n", ret);
      }
    } 
  } else {
      servo2.write(manual_angle);
  }
}
// Servo servo1, servo2;
// int pin1 = 0, pin2 = 15;

// rcl_subscription_t sub1, sub2;
// std_msgs__msg__Float32 msg1, msg2;
// rcl_node_t node;
// rcl_allocator_t allocator;
// rclc_support_t support;
// rclc_executor_t executor;

// int minUs = 500;
// int maxUs = 2500;

// void cb1(const void *msgin) {
//   float cmd = ((std_msgs__msg__Float32*)msgin)->data;
//   servo1.writeMicroseconds(1500 + cmd * 2);
// }
// void cb2(const void *msgin) {
//   float cmd = ((std_msgs__msg__Float32*)msgin)->data;
//   servo2.writeMicroseconds(1500 + cmd * 2);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(2000);

//   // ==== Wi-Fi 接続 ====
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to WiFi");
//   while(WiFi.status() != WL_CONNECTED) {
//     Serial.print(".");
//     delay(500);
//   }
//   Serial.println("\nWiFi connected!");
//   Serial.println(WiFi.localIP());

//   // micro-ROS エージェントに接続
//   set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
//   delay(2000);

//   rclc_support_init(&support, 0, NULL, &allocator);
//   rclc_node_init_default(&node, "esp_servo_node", "", &support);

//   rclc_subscription_init_default(&sub1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "servo1/cmd");
//   rclc_subscription_init_default(&sub2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "servo2/cmd");

//   rclc_executor_init(&executor, &support.context, 2, &allocator);
//   rclc_executor_add_subscription(&executor, &sub1, &msg1, &cb1, ON_NEW_DATA);
//   rclc_executor_add_subscription(&executor, &sub2, &msg2, &cb2, ON_NEW_DATA);

//   servo1.attach(pin1);
//   servo2.attach(pin2);
//   allocator = rcl_get_default_allocator();
// }

// void loop() {
//   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
// }