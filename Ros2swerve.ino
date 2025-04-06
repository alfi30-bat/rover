#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <cstdio>
#include <cstdlib>
#include <math.h>

// Motor and wheel configuration
#define TICKS_PER_REV 2048
#define MAX_PWM 255
#define WHEEL_RADIUS 123
#define TRACK_WIDTH 732.28
#define WHEEL_BASE 820
#define PI 3.14159265


#define FL_ENC_A 0
#define FL_ENC_B 1
#define FR_ENC_A 15
#define FR_ENC_B 14
#define RL_ENC_A 21
#define RL_ENC_B 20
#define RR_ENC_A 28
#define RR_ENC_B 29
#define SFL_ENC_A 7
#define SFL_ENC_B 8
#define SFR_ENC_A 16
#define SFR_ENC_B 17
#define SRL_ENC_A 25
#define SRL_ENC_B 24
#define SRR_ENC_A 34
#define SRR_ENC_B 35

// --- Motor Driver Pins ---
#define FL_DRIVE_PWM 3
#define FL_DRIVE_DIR 4
#define FR_DRIVE_PWM 5
#define FR_DRIVE_DIR 6
#define RL_DRIVE_PWM 9
#define RL_DRIVE_DIR 10
#define RR_DRIVE_PWM 11
#define RR_DRIVE_DIR 12

 #include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor1(PWM_DIR, 3, 4);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 5, 6); 
CytronMD motor3(PWM_DIR, RL_DRIVE_PWM, RL_DRIVE_DIR);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor4(PWM_DIR, RR_DRIVE_PWM, RR_DRIVE_DIR); 

#define FL_STEER_PWM 18
#define FL_STEER_DIR 19
#define FR_STEER_PWM 36
#define FR_STEER_DIR 37
#define RL_STEER_PWM 22
#define RL_STEER_DIR 23
#define RR_STEER_PWM 2
#define RR_STEER_DIR 33
const int ledPin = 13;

// ROS 2 Node and Subscription
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t odom_pub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
const int K_P = 278;
double basePWM=0;
double vx=0,vy=0,omega=0;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
// Odometry variables
nav_msgs__msg__Odometry odom_msg;
float x = 0.0, y = 0.0, theta = 0.0;
unsigned long last_time = 0;

// Motor parameters
int ticks_left_front = 0, ticks_right_front = 0, ticks_left_rear = 0, ticks_right_rear = 0;


// Wheel speed and angle
double fl_speed, fr_speed, rl_speed, rr_speed;
double fl_angle, fr_angle, rl_angle, rr_angle;



// --- Motor Control Functions ---
void setDriveMotor(CytronMD& motor, int pwm_pin, int dir_pin, double speed) {
    basePWM = K_P * fabs(speed) + b;
    speed = constrain(basePWM, 0, MAX_PWM);
    // Adjust motor direction and speed based on the PWM signal
    if (vx >= 0 && vy==0 && omega==0) {
        digitalWrite(dir_pin, HIGH);     // Set motor speed forward
 // Forward direction
        motor.setSpeed(speed);   
    } else if (vx < 0 && vy==0 && omega==0) {
        digitalWrite(dir_pin, LOW);  // Reverse direction
  // Reverse direction
        motor.setSpeed(-speed);  
            // Set motor speed in reverse (assuming negative speed means reverse)
    } else   if (vy >= 0 && vx==0 && omega==0) {
        digitalWrite(dir_pin, HIGH);     // Set motor speed forward
 // Forward direction
        motor.setSpeed(speed);   
    } else if (vy < 0 && vx==0 && omega==0) {
        digitalWrite(dir_pin, LOW);  // Reverse direction
  // Reverse direction
        motor.setSpeed(-speed);  
            // Set motor speed in reverse (assuming negative speed means reverse)
    } else    if (omega >= 0 && vy==0 && vx==0) {
        digitalWrite(dir_pin, HIGH);     // Set motor speed forward
 // Forward direction
        motor.setSpeed(speed);   
    } else if (omega < 0 && vy==0 && vx==0) {
        digitalWrite(dir_pin, LOW);  // Reverse direction
  // Reverse direction
        motor.setSpeed(-speed);  
            // Set motor speed in reverse (assuming negative speed means reverse)
    }  else {
        motor.setSpeed(0);           // Stop the motor if speed is zero
    }

    // Ensure the speed is within bounds
}


void setSteerMotor(int pwm_pin, int dir_pin, double target_angle) {
    // Assuming the current angle is a global variable or can be retrieved.
    static double current_angle = 0.0;  // Replace with actual current angle if available.
    
    // Calculate the error between the current angle and the target angle
    double error = target_angle - current_angle;
    
    // Check if we are close enough to the target angle
    const double tolerance = 0.05; // Define a small tolerance to stop the motor when close enough

    if (fabs(error) > tolerance) {
        // Move the motor towards the target angle
        int pwm_value = map(target_angle * 100, -PI * 100, PI * 100, 0, MAX_PWM);
        pwm_value = constrain(pwm_value, 0, MAX_PWM);

        // Apply PWM to the motor
        if (vx!=0)
        analogWrite(pwm_pin, pwm_value);
        else
        analogWrite(pwm_pin, 0);

        // Determine the direction of the motor
        if (error > 0) {
            digitalWrite(dir_pin, HIGH);  // Rotate in one direction
        } else {
            digitalWrite(dir_pin, LOW);   // Rotate in the opposite direction
        }

        // Optionally, update the current angle based on motor movement (if you have an encoder or some feedback)
        current_angle += error * 0.1;  // Adjust based on your control system
    } else {
        // Stop the motor if we're close enough to the target angle
        analogWrite(pwm_pin, 0);
    }
}




// --- Swerve Kinematics ---
void calculateSwerve(double vx, double vy, double omega) {
    double A = vx - omega * (WHEEL_BASE / 2);
    double B = vx + omega * (WHEEL_BASE / 2);


    // Compute wheel speeds
    fl_speed = sqrt(A * A + vy * vy);
    fr_speed = sqrt(B * B +  vy * vy);
    rl_speed = sqrt(A * A + vy * vy);
    rr_speed = sqrt(B * B + vy * vy);

    // Compute wheel angles (steering)
    fl_angle = atan2(vy, A);
    fr_angle = atan2(vy, B);
    rl_angle = atan2(vy, A);
    rr_angle = atan2(vy, B);

    // Normalize speed if needed
    double max_speed = max(max(fl_speed, fr_speed), max(rl_speed, rr_speed));
    if (max_speed > MAX_PWM) {
        fl_speed = (fl_speed / max_speed) * MAX_PWM;
        fr_speed = (fr_speed / max_speed) * MAX_PWM;
        rl_speed = (rl_speed / max_speed) * MAX_PWM;
        rr_speed = (rr_speed / max_speed) * MAX_PWM;
    }
}

// --- ROS Callback ---
geometry_msgs__msg__Twist cmd_vel_msg;

void cmd_vel_callback(const void *msg_in) {
    const geometry_msgs__msg__Twist *cmd = (const geometry_msgs__msg__Twist *)msg_in;

     vx = cmd->linear.x;   // Joystick forward/backward
     vy = cmd->linear.y;   // Joystick sideways strafing
     omega = cmd->angular.z; // Joystick rotation
    
    calculateSwerve(vx, vy, omega);
    if(basePWM<0){
  digitalWrite(ledPin, HIGH);   // set the LED on
    }else{
  digitalWrite(ledPin, LOW);   // set the LED on

    }
    // Set Drive Motors (wheel speeds)
    setDriveMotor(motor1, FL_DRIVE_PWM, FL_DRIVE_DIR, fl_speed);
    setDriveMotor(motor2, FR_DRIVE_PWM, FR_DRIVE_DIR, fr_speed);
    setDriveMotor(motor3, RL_DRIVE_PWM, RL_DRIVE_DIR, rl_speed);
    setDriveMotor(motor4, RR_DRIVE_PWM, RR_DRIVE_DIR, rr_speed);

    // Set Steering Motors (wheel angles)
    setSteerMotor(FL_STEER_PWM, FL_STEER_DIR, fl_angle);
    setSteerMotor(FR_STEER_PWM, FR_STEER_DIR, fr_angle);
    setSteerMotor(RL_STEER_PWM, RL_STEER_DIR, rl_angle);
    setSteerMotor(RR_STEER_PWM, RR_STEER_DIR, rr_angle);

    // Stop drive motors when joystick is neutral
    if (vx == 0 && vy == 0 && omega == 0) {
        analogWrite(FL_DRIVE_PWM, 0);
        analogWrite(FR_DRIVE_PWM, 0);
        analogWrite(RL_DRIVE_PWM, 0);
        analogWrite(RR_DRIVE_PWM, 0);
        analogWrite(FL_STEER_PWM, 0);
        analogWrite(FR_STEER_PWM, 0);
        analogWrite(RL_STEER_PWM, 0);
        analogWrite(RR_STEER_PWM, 0);        
    }
}

void computeOdometry() {

  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // Time in seconds
  
  // Calculate velocities for each wheel
  float v_LF = ticks_left_front * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  float v_RF = ticks_right_front * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  float v_LR = ticks_left_rear * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  float v_RR = ticks_right_rear * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  
  // Robot's velocities in robot frame (holonomic robot)
  // Assuming wheel velocities are arranged to affect x and y linear velocities and rotational velocity
  float v_x = (v_LF + v_RF + v_LR + v_RR) / 4.0;  // Average of all wheels for translational velocity
  float v_y = 0; // Can be adjusted for holonomic systems with appropriate wheel placement
  float omega = (v_RR - v_LF + v_RF - v_LR) / WHEEL_BASE; // A simple approach for rotational velocity
  
  // Update position and orientation using kinematics
  x += v_x * cos(theta) * dt;
  y += v_x * sin(theta) * dt;
  theta += omega * dt;
  
  // Prepare the odometry message
  odom_msg.header.stamp.sec = current_time / 1000;
  odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta / 2);
  odom_msg.pose.pose.orientation.w = cos(theta / 2);
  
  // Publish the odometry message
  (void)rcl_publish(&odom_pub, &odom_msg, NULL);
  
  // Reset ticks for the next loop
  ticks_left_front = 0;
  ticks_right_front = 0;
  ticks_left_rear = 0;
  ticks_right_rear = 0;
  
  last_time = current_time;
  
  delay(100);  // Update rate (in milliseconds)
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    set_microros_transports();
  pinMode(ledPin, OUTPUT);

    pinMode(FL_DRIVE_DIR, OUTPUT);
    pinMode(FR_DRIVE_DIR, OUTPUT);
    pinMode(RL_DRIVE_DIR, OUTPUT);
    pinMode(RR_DRIVE_DIR, OUTPUT);

    pinMode(FL_STEER_DIR, OUTPUT);
    pinMode(FR_STEER_DIR, OUTPUT);
    pinMode(RL_STEER_DIR, OUTPUT);
    pinMode(RR_STEER_DIR, OUTPUT);


    // Encoder pin setup
    pinMode(FL_ENC_A, INPUT);
    pinMode(FL_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), encoderLeftFront, CHANGE);

    pinMode(FR_ENC_A, INPUT);
    pinMode(FR_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), encoderRightFront, CHANGE);

    pinMode(RL_ENC_A, INPUT);
    pinMode(RL_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RL_ENC_A), encoderLeftRear, CHANGE);

    pinMode(RR_ENC_A, INPUT);
    pinMode(RR_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RR_ENC_A), encoderRightRear, CHANGE);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "swerve_drive_node", "", &support);

    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA);

  }
    // set the LED on
void encoderLeftFront() { ticks_left_front++; }
void encoderRightFront() { ticks_right_front++; }
void encoderLeftRear() { ticks_left_rear++; }
void encoderRightRear() { ticks_right_rear++; }

// --- Loop ---
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    computeOdometry();
    //set_pwm();

}
