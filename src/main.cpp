//#######################################################################################################
// Name:             main.cpp
// Purpose:          Jupiter Robot ESP32 firmware
// Description:      Robot motor drivers, IMU, LED are controlled using PID and communicates to Host PC
//                   using Micro-ROS.  This firmware reads cmd_vel msgs from ROS2 host and publishes
//                   imu/data and odom/unfiltered msgs back to the host so that ROS2 Navigation can compute
//                   the robots position and orientation and determine velocity feedback to the ESP32
//                   Also included are other modules that drive the attached OLED display and Onboard LED
//                   to indicate when the Robot is listening to voice commands.
// Related Files:    this firmware is built to compile on VS CODE using the PLATFORMIO plugin     
// Author:           logan naidoo, south africa, 2024
//########################################################################################################

#include <Arduino.h>                            // needed if using Platformio and VS Code IDE
#include <micro_ros_platformio.h>               // use if using platformio, otherwise #include <miro_ros_arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <utility/imumaths.h>
#include <nav_msgs/msg/odometry.h>

// Include the local header files
#include "jupiter_config.h"
#include "imu_bno055.h"
#include "encoder.h"
#include "kinematics.h"
// #include "odometry.h"
#include "pid_controller.h"

#include "motor.h"



// ROS-related variables
rcl_publisher_t odom_publisher;

// Declare the cmd_vel subscriber and twist message variable
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// Declare the IMU publisher
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;  

// Declare the encoder publisher oject and message array
rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

// Declare the ROS & micro-ROS node interfaces
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Setup Motor Instances
Motor motor1(MOTOR1_PWM, MOTOR1_DIR, 0, PWM_FREQ, PWM_BITS); // attach to pwm chan 0
Motor motor2(MOTOR2_PWM, MOTOR2_DIR, 1, PWM_FREQ, PWM_BITS); // attach to pwm chan 1
Motor motor3(MOTOR3_PWM, MOTOR3_DIR, 2, PWM_FREQ, PWM_BITS); // attach to pwm chan 2
Motor motor4(MOTOR4_PWM, MOTOR4_DIR, 3, PWM_FREQ, PWM_BITS); // attach to pwm chan 3


// Setup Wheel Ecoders for each Motor
Encoder motor1_encoder(MOTOR1_ENC_A, MOTOR1_ENC_B, ENCODER_CPR);
Encoder motor2_encoder(MOTOR2_ENC_A, MOTOR2_ENC_B, ENCODER_CPR);
Encoder motor3_encoder(MOTOR3_ENC_A, MOTOR3_ENC_B, ENCODER_CPR);
Encoder motor4_encoder(MOTOR4_ENC_A, MOTOR4_ENC_B, ENCODER_CPR);


Kinematics kinematics(WHEEL_RADIUS, WHEEL_SEPARATION);
// Odometry odometry(&odom_publisher, WHEEL_RADIUS, WHEEL_SEPARATION);

PIDController pid_front_left(PID_KP, PID_KI, PID_KD);
PIDController pid_front_right(PID_KP, PID_KI, PID_KD);
PIDController pid_back_left(PID_KP, PID_KI, PID_KD);
PIDController pid_back_right(PID_KP, PID_KI, PID_KD);

float target_linear_velocity = 0;
float target_angular_velocity = 0;

void setMotorSpeed(int fl_speed, int fr_speed, int bl_speed, int br_speed) {
    // Set all motor PWM outputs to zero to stop the motors
    motor1.setSpeed(fl_speed);
    motor2.setSpeed(fr_speed);
    motor3.setSpeed(bl_speed);
    motor4.setSpeed(br_speed);
    if (fr_speed == 0)
        digitalWrite(ESP32_LED, LOW);
    else
        digitalWrite(ESP32_LED, HIGH);
}

void cmdVelCallback(const void *msg_in) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    target_linear_velocity = msg->linear.x;
    target_angular_velocity = msg->angular.z;

    

    double front_left_speed, front_right_speed, back_left_speed, back_right_speed;

    // Compute the wheel speeds based on the cmd_vel input
    kinematics.computeWheelSpeeds(target_linear_velocity, target_angular_velocity, front_left_speed, front_right_speed, back_left_speed, back_right_speed);

    
    // // Set motor speeds for all four motors
    setMotorSpeed(front_left_speed, front_right_speed, back_left_speed, back_right_speed); 

}

void timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    
    // Get & Publish the IMU message
    get_imu_data(&imu_msg);
    rcl_ret_t ret_imu_ok = rcl_publish(&imu_publisher, &imu_msg, NULL);

    // ---------------Encoder pub test ------------------------------
    // Update encoder message
    encoder_msg.data.data[0] = int32_t (motor1_encoder.getCount());
    encoder_msg.data.data[1] = int32_t (motor2_encoder.getCount());
    encoder_msg.data.data[2] = int32_t (motor3_encoder.getCount());
    encoder_msg.data.data[3] = int32_t (motor4_encoder.getCount());
    // Publish encoder data
    rcl_ret_t ret_enc_ok = rcl_publish(&encoder_publisher, &encoder_msg, NULL);

}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);

    // Set initial PWM values to zero to prevent motors from moving at startup
    setMotorSpeed(0, 0, 0, 0);

    // Initialise Encoders and attach interrupts and reset the counters
    motor1_encoder.begin();
    motor2_encoder.begin();
    motor3_encoder.begin();
    motor4_encoder.begin();
    delay(3000);            // delay to allow wheels spin on startup to stop & reset counters
    motor1_encoder.reset();
    motor2_encoder.reset();
    motor3_encoder.reset();
    motor4_encoder.reset();


    // ------------------END NEW GPT CODE ----------------------

    // Set Micro-ROS transport
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Create init_options and support
    rclc_support_init(&support, 0, NULL, &allocator);

    // ---------------------  NEW GPT CODE -------------------------------
    rcl_node_t node = rcl_get_zero_initialized_node();
    // ---------------------- END NEW GPT CODE ---------------------------

    // Create node
    rclc_node_init_default(&node, "esp32_node", "", &support);


    // Create IMU publisher
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data");

    // Create cmd_vel subscriber
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Create odometry publisher
    rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom/unfiltered");

    // ---------------------------Encoder TEST --------------------------
    
    // Create encoder publisher
    rclc_publisher_init_default(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "wheel_encoders");

    // Initialize encoder message
    encoder_msg.data.size = 4;
    encoder_msg.data.capacity = 4;
    encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));
    // ___________________ end encoder test ______________________________
    
    // Create timer
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100), // Publish IMU and any other callback data every 100 ms
        timerCallback);
    
    


    // ---------------------  NEW GPT CODE -------------------------------
    // executor = rclc_executor_get_zero_initialized_executor();
    // ---------------------- END NEW GPT CODE ---------------------------

    // Create executor which only handles timer and subscriber callbacks
    rclc_executor_init(&executor, &support.context, 2, &allocator);  // adjust the number of handles for each callback added
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);

    // Initialise (setup) IMU and OLED
    setup_imu(&imu_msg);
    setup_oled_display();

    // Set up LED pin
    pinMode(ESP32_LED, OUTPUT);
    digitalWrite(ESP32_LED, LOW);

    
}

void loop() {
    float dt = 1.0 / CONTROL_LOOP_HZ;

    // Spin the ROS 2 executor to handle callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // was 100

    // Stop motors if both target velocities are zero
    if (target_linear_velocity == 0 && target_angular_velocity == 0) {
        setMotorSpeed(0, 0, 0, 0);
    } 

    // Update odometry
    // odometry.update(fl_velocity, fr_velocity, bl_velocity, br_velocity, dt);
    // Get the odometry message
    // nav_msgs__msg__Odometry odom_msg = odometry.getOdometryMsg();

    // Publish the Odometry message
    // rcl_ret_t ret_odom_ok = rcl_publish(&odom_publisher, &odom_msg, NULL);

    // Calculate PID outputs
    // WheelSpeeds target_speeds = kinematics.inverseKinematics(target_linear_velocity, target_angular_velocity);
    // float fl_output = pid_front_left.calculate(target_speeds.front_left, fl_velocity, dt);
    // float fr_output = pid_front_right.calculate(target_speeds.front_right, fr_velocity, dt);
    // float bl_output = pid_back_left.calculate(target_speeds.back_left, bl_velocity, dt);
    // float br_output = pid_back_right.calculate(target_speeds.back_right, br_velocity, dt);



    // Drive motors using PWM and direction control
    // setMotorSpeed(fl_output, fr_output, bl_output, br_output); 
 
    delay(1000 / CONTROL_LOOP_HZ);
}