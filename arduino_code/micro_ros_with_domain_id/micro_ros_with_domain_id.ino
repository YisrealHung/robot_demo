#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// 定義四顆馬達的引腳
#define motor1Pin1 16  // 左前輪馬達引腳1
#define motor1Pin2 4 // 左前輪馬達引腳2
#define motor2Pin1 17 // 左後輪馬達引腳1
#define motor2Pin2 13 // 左後輪馬達引腳2
#define motor3Pin1 26 // 右前輪馬達引腳1
#define motor3Pin2 33 // 右前輪馬達引腳2
#define motor4Pin1 25 // 右後輪馬達引腳1
#define motor4Pin2 32 // 右後輪馬達引腳2

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void motor_control(int ls, int rs) {
  if (ls >=0) {
    analogWrite(motor1Pin1, ls);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, ls);
    analogWrite(motor2Pin2, 0);
  }
  else {
    ls = ls*-1;
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, ls);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, ls);
  }

  if (rs >=0) {
    analogWrite(motor3Pin1, rs);
    analogWrite(motor3Pin2, 0);
    analogWrite(motor4Pin1, rs);
    analogWrite(motor4Pin2, 0);
  }
  else {
    rs = rs*-1;
    analogWrite(motor3Pin1, 0);
    analogWrite(motor3Pin2, rs);
    analogWrite(motor4Pin1, 0);
    analogWrite(motor4Pin2, rs);
  }

}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  int lx = map(int(msg->linear.x*100), -50, 50, -250, 250);
  lx = constrain(lx, -250, 250);
  int az = map(int(msg->angular.z*100), -100, 100, -250, 250);
  az = constrain(az, -250, 250);

  int Lspeed = lx - az;
  int Rspeed = lx + az;
  motor_control(Lspeed, Rspeed);
}

void setup() {
  // Initialize motor control pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // set domain id
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 69);
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
