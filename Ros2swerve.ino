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
#include <std_msgs/msg/int32_multi_array.h>

struct Motor {
    int pwm_pin;
    int dir_pin;
    volatile int32_t* ticks;   // Pointer to the encoder tick count
    int32_t startTicks;
    int32_t targetTicks;
    bool moving;
};
float lastCommandedAngle = 9999.0; // Start with an impossible value.


// Variables for odometry
float x = 0.0;      // Current x position
float y = 0.0;      // Current y position
float theta = 0.0;  // Current orientation

unsigned long last_time = 0;  // Previous time for calculation

// Array to store encoder tick counts for drive wheels
int32_t drive_ticks[4] = {0}; // FL, FR, RL, RR

// Array to store encoder tick counts for steering wheels
 int32_t steer_ticks[4] = {0}; // FL, FR, RL, RR

// MicroROS Publishers
rcl_publisher_t tick_publisher;
rcl_publisher_t odom_publisher;

// ROS Messages
std_msgs__msg__Int32MultiArray tick_msg;
nav_msgs__msg__Odometry odom_msg;

// Motor and wheel configuration
#define TICKS_PER_REV 2048
#define MAX_PWM 255
#define WHEEL_RADIUS 123
#define TRACK_WIDTH 732.28
#define WHEEL_BASE 820
#define PI 3.14159265
#define STEER_TICKS_PER_REV 65172           // quadrature counts per full rev (360°)
#define FULL_ANGLE 360.0f
#define HALF_ANGLE 180.0f
#define SPEED_PWM  153             // ~60% duty
         // 5 s timeout

// Encoder pin definitions for drive and steering wheels (replace as per setup)
#define FL_DRIVE_ENC_A 0
#define FL_DRIVE_ENC_B 1
#define FR_DRIVE_ENC_A 15
#define FR_DRIVE_ENC_B 14
#define RL_DRIVE_ENC_A 21
#define RL_DRIVE_ENC_B 20
#define RR_DRIVE_ENC_A 28
#define RR_DRIVE_ENC_B 29

#define FL_STEER_ENC_A 7
#define FL_STEER_ENC_B 8
#define FR_STEER_ENC_A 16
#define FR_STEER_ENC_B 17
#define RL_STEER_ENC_A 25
#define RL_STEER_ENC_B 24
#define RR_STEER_ENC_A 34
#define RR_STEER_ENC_B 35


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
double basepwm=0;
double vx=0,vy=0,omega=0;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
// Odometry variables

bool forward;
bool busy;

// Motor parameters
int ticks_left_front = 0, ticks_right_front = 0, ticks_left_rear = 0, ticks_right_rear = 0;


// Wheel speed and angle
double fl_speed, fr_speed, rl_speed, rr_speed;
double fl_angle, fr_angle, rl_angle, rr_angle;


void setDriveMotor(int pwm_pin, int dir_pin, double speed) {
    if (speed >= 0) {
        digitalWrite(dir_pin, HIGH);
        analogWrite(pwm_pin, 180);
    } else if(speed<0) {
        digitalWrite(dir_pin, LOW);
        speed = -speed;
        analogWrite(pwm_pin, -180);
    }else{
      analogWrite(pwm_pin, 0);
    }
    speed = constrain(speed, 0, MAX_PWM);
    //analogWrite(pwm_pin, speed);
}


void startMotor(Motor &motor, int32_t delta) {
    noInterrupts();
    motor.startTicks = *(motor.ticks);
    motor.targetTicks = motor.startTicks + delta;
    interrupts();

    bool forward = (delta > 0);
    digitalWrite(motor.dir_pin, forward ? HIGH : LOW);
    analogWrite(motor.pwm_pin, SPEED_PWM);
    motor.moving = true;
}
// Assume you have defined your motors array with proper initialization:
Motor motors[4] = {
    { FL_STEER_PWM, FL_STEER_DIR, &steer_ticks[0], 0, 0, false },
    { FR_STEER_PWM, FR_STEER_DIR, &steer_ticks[1], 0, 0, false },
    { RL_STEER_PWM, RL_STEER_DIR, &steer_ticks[2], 0, 0, false },
    { RR_STEER_PWM, RR_STEER_DIR, &steer_ticks[3], 0, 0, false }
};
float mapRange(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Function: Initiate motor movement by a given delta of encoder ticks.
// --------------------
void moveBy(int32_t delta, int pwm_pin, int dir_pin, volatile int32_t *msteer_ticks, Motor &motor) {
    if (delta == 0) { 
        Serial.println(F("↔ No movement needed."));
        return;
    }
    // Explanation: If the change is zero, nothing is done.

    noInterrupts();
    motor.startTicks = *msteer_ticks;
    motor.targetTicks = motor.startTicks + delta;
    interrupts();
    // Explanation: Record the starting ticks and compute the target tick value.

    //bool forward = (delta > 0);
    digitalWrite(dir_pin, forward ? HIGH : LOW);
    if (forward == HIGH)
        analogWrite(pwm_pin, SPEED_PWM);
    else
        analogWrite(pwm_pin, -SPEED_PWM);      
    motor.moving = true;
    // Explanation: Set the motor direction and start that motor at a certain speed.
}

// Assuming STEER_TICKS_PER_REV is defined elsewhere (e.g., 1024)
// and the Motor structure, steer_ticks[] array, and motors[] array are already declared.

const int32_t TICK_DEADBAND = 3;  // Threshold for correcting small errors.
const int32_t MIN_STEER_TICKS = -30000;                  // Corresponds to -90°.
const int32_t MAX_STEER_TICKS = 30000;  // Corresponds to +90°.

// Synchronize all four steering motors so that their encoder ticks are close to each other.
// This function computes the average encoder tick of all motors and commands any motor
// that is too far away from that average (beyond the deadband) to move toward it.
void syncSteerMotors() {
  int32_t sum = 0;
  // Get the current tick readings from all four motors.
  for (int i = 0; i < 3; i++) {
    noInterrupts();
    int32_t t = steer_ticks[i];
    interrupts();
    sum += t;
  }
  int32_t avg = sum / 3;
  Serial.print("Synchronizing. Average tick = ");
  Serial.println(avg);
  
  // For each motor, if its tick reading differs by more than TICK_DEADBAND from the average,
  // command that motor to adjust.
  for (int i = 0; i < 3; i++) {
    noInterrupts();
    int32_t current = steer_ticks[i];
    interrupts();
    int32_t delta = avg - current;
    if (abs(delta) >= TICK_DEADBAND) {
      // Compute a new target but clamp it to allowed range.
      int32_t target = current + delta;
      if (target < MIN_STEER_TICKS) target = MIN_STEER_TICKS;
      if (target > MAX_STEER_TICKS) target = MAX_STEER_TICKS;
      delta = target - current;  // Re-calc the final delta.
      if (delta != 0) {
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(" synchronizing by delta: ");
        Serial.println(delta);
        moveBy(delta, motors[i].pwm_pin, motors[i].dir_pin, motors[i].ticks, motors[i]);
      }
    }
  }
}

// Set the steering motors to the desired target angle.
// The input 'anglee' is in degrees and is clamped to [-90, 90].
// (Note: With our mapping, -90° → 0 ticks and +90° → STEER_TICKS_PER_REV.
//  Thus, 0° corresponds to STEER_TICKS_PER_REV/2.)
void setSteerAngle(int anglee) {
  // Make sure the desired angle is within bounds.
  if (anglee > 90) anglee = 90;
  if (anglee < -90) anglee = -90;
  
  // Convert the target angle to an absolute encoder tick value.
  int32_t desiredTicks = (int32_t) round((anglee + 90.0) * STEER_TICKS_PER_REV / 180.0);
  Serial.print("Setting steer angle ");
  Serial.print(anglee);
  Serial.print("° (mapped to ticks: ");
  Serial.print(desiredTicks);
  Serial.println(")");
  
  // For each steering motor, compute the difference between desired and current ticks,
  // and command the motor to move if the difference is significant.
  for (int i = 0; i < 4; i++) {
    noInterrupts();
    int32_t current = steer_ticks[i];
    interrupts();
    int32_t delta = desiredTicks - current;
    if (abs(delta) >= TICK_DEADBAND) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" moving by delta: ");
      Serial.println(delta);
      moveBy(delta, motors[i].pwm_pin, motors[i].dir_pin, motors[i].ticks, motors[i]);
    } else {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println(" within deadband; no movement needed.");
    }
  }
}

// Example control function that might be called from your joystick callback.
// This function first synchronizes the steering motors, then moves them to the desired target.
// If there is no joystick input, then 'anglee' should be zero (center).
void updateSteering(int anglee) {
  // First, synchronize the steering motors so their encoder readings are aligned.
  syncSteerMotors();
  
  // Next, command all steering motors to reach the target angle.
  setSteerAngle(anglee);
}


geometry_msgs__msg__Twist cmd_vel_msg;
// --- Example ROS Callback snippet ---
// (Assume that mapping from joystick input to 'anglee' has been done elsewhere.)
void cmd_vel_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *cmd = (const geometry_msgs__msg__Twist *)msg_in;
  
  // Map joystick input (assumed here to be cmd->linear.y) to an angle (–90° to 90°):
  float vy = cmd->linear.y;   // Expected range: -1.0 to 1.0.
  float anglee = mapRange(vy, -1.0, 1.0, -90.0, 90.0);
  
  // For a neutral joystick (or very near zero), you want the steer motors to center (0°).
  if (fabs(vy) < 0.05) {
    anglee = 0; // Center position.
  }
  

  // (Optional) Ensure you only call steering if the command has changed significantly.
  // Otherwise, repeated identical commands might trigger unnecessary movements.
  // Here you could compare anglee with a stored "lastCommandedAngle" and only update if changed.
  
  Serial.print("Joystick mapped to steer angle: ");
  Serial.println(anglee);
  
  // Update the steering system.
  updateSteering((int)anglee);


    if (vy>0)
    forward = HIGH;
    else
    forward=LOW;

     vx = cmd->linear.x;   // Joystick forward/backward

     omega = cmd->angular.z; // Joystick rotation
    basepwm = K_P *vx +b;

  checkSteerLimits();
    if(vx<0){
  digitalWrite(ledPin, HIGH);   // set the LED on
    }else{
  digitalWrite(ledPin, LOW);   // set the LED on
    }
    if(vx>0 && vy>0 || vx>0 && vy<0){
        busy=HIGH;
    // Set Drive Motors (wheel speeds)
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, basepwm);
    setSteerAngle(anglee);
    }else if(vx<0 && vy<0 || vx<0 && vy>0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, -basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, -basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, -basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, -basepwm);
    setSteerAngle(anglee);      
        busy=HIGH;

    }else if(vx>0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, basepwm);
    
    }else if(vx<0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, -basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, -basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, -basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, -basepwm);
    }else if(vy>0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, -basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, -basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, -basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, -basepwm);
    setSteerAngle(anglee);  
        busy=HIGH;

    }else if(vy<0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, basepwm);
    setSteerAngle(anglee);  
        busy=HIGH;

    }else if(omega>0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, -basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, -basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, -basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, -basepwm);
    setSteerAngle(60);      
    }else if(omega<0){
    setDriveMotor(FL_DRIVE_PWM, FL_DRIVE_DIR, -basepwm);
    setDriveMotor(FR_DRIVE_PWM, FR_DRIVE_DIR, -basepwm);
    setDriveMotor(RL_DRIVE_PWM, RL_DRIVE_DIR, -basepwm);
    setDriveMotor(RR_DRIVE_PWM, RR_DRIVE_DIR, -basepwm);
    setSteerAngle(60);   
    }else{
        analogWrite(FL_DRIVE_PWM, 0);
        analogWrite(FR_DRIVE_PWM, 0);
        analogWrite(RL_DRIVE_PWM, 0);
        analogWrite(RR_DRIVE_PWM, 0);
        analogWrite(FL_STEER_PWM, 0);
        analogWrite(FR_STEER_PWM, 0);
        analogWrite(RL_STEER_PWM, 0);
        analogWrite(RR_STEER_PWM, 0);      
        busy=LOW;

    }


}

// Last known states for quadrature decoding (for drive wheels)
int last_drive_val[4] = {HIGH, HIGH, HIGH, HIGH};

// Last known states for quadrature decoding (for steer wheels)
int last_steer_val[4] = {HIGH, HIGH, HIGH, HIGH};

// Interrupt service routines for drive and steer wheels
void encoderDriveFL() { update_wheel_tick(0, FL_DRIVE_ENC_A, FL_DRIVE_ENC_B, drive_ticks, last_drive_val); }
void encoderDriveFR() { update_wheel_tick(1, FR_DRIVE_ENC_A, FR_DRIVE_ENC_B, drive_ticks, last_drive_val); }
void encoderDriveRL() { update_wheel_tick(2, RL_DRIVE_ENC_A, RL_DRIVE_ENC_B, drive_ticks, last_drive_val); }
void encoderDriveRR() { update_wheel_tick(3, RR_DRIVE_ENC_A, RR_DRIVE_ENC_B, drive_ticks, last_drive_val); }

void encoderSteerFL() { update_wheel_tick(0, FL_STEER_ENC_A, FL_STEER_ENC_B, steer_ticks, last_steer_val); }
void encoderSteerFR() { update_wheel_tick(1, FR_STEER_ENC_A, FR_STEER_ENC_B, steer_ticks, last_steer_val); }
void encoderSteerRL() { update_wheel_tick(2, RL_STEER_ENC_A, RL_STEER_ENC_B, steer_ticks, last_steer_val); }
void encoderSteerRR() { update_wheel_tick(3, RR_STEER_ENC_A, RR_STEER_ENC_B, steer_ticks, last_steer_val); }

// Update tick count for the specified motor
void update_wheel_tick(int index, int encoder_input1, int encoder_input2, int32_t *ticks, int *last_val) {

  //static uint8_t last = 0;
  uint8_t a = digitalRead(encoder_input1), b = digitalRead(encoder_input2);
  uint8_t curr = (a << 1) | b;
  uint8_t sum  = ( last_val[index] << 2) | curr;
  if (sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) ticks[index]++;
  else if (sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) ticks[index]--;
  last_val[index] = curr;
}


// Publish tick counts
void publish_ticks() {
    tick_msg.data.size = 8;
    tick_msg.data.data = (int32_t*)malloc(sizeof(int32_t) * 8);

    for (int i = 0; i < 4; i++) {
        tick_msg.data.data[i] = drive_ticks[i];
        tick_msg.data.data[i + 4] = steer_ticks[i];
    }

    rcl_publish(&tick_publisher, &tick_msg, NULL);
    free(tick_msg.data.data);
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


    // Drive Encoder Pin Setup
    pinMode(FL_DRIVE_ENC_A, INPUT);
    pinMode(FL_DRIVE_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FL_DRIVE_ENC_A), encoderDriveFL, CHANGE);

    pinMode(FR_DRIVE_ENC_A, INPUT);
    pinMode(FR_DRIVE_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FR_DRIVE_ENC_A), encoderDriveFR, CHANGE);

    pinMode(RL_DRIVE_ENC_A, INPUT);
    pinMode(RL_DRIVE_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RL_DRIVE_ENC_A), encoderDriveRL, CHANGE);

    pinMode(RR_DRIVE_ENC_A, INPUT);
    pinMode(RR_DRIVE_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RR_DRIVE_ENC_A), encoderDriveRR, CHANGE);

    // Steer Encoder Pin Setup
    pinMode(FL_STEER_ENC_A, INPUT);
    pinMode(FL_STEER_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FL_STEER_ENC_A), encoderSteerFL, CHANGE);

    pinMode(FR_STEER_ENC_A, INPUT);
    pinMode(FR_STEER_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FR_STEER_ENC_A), encoderSteerFR, CHANGE);

    pinMode(RL_STEER_ENC_A, INPUT);
    pinMode(RL_STEER_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RL_STEER_ENC_A), encoderSteerRL, CHANGE);

    pinMode(RR_STEER_ENC_A, INPUT);
    pinMode(RR_STEER_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(RR_STEER_ENC_A), encoderSteerRR, CHANGE);


    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "swerve_drive_node", "", &support);

    rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");


    rclc_publisher_init_default(&tick_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "motor_ticks");

    tick_msg.data.capacity = 8;
    tick_msg.data.size = 0;
    tick_msg.data.data = NULL;

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA);

  }


    int32_t savedSteerTicks[4] = {0, 0, 0, 0};// set the LED on
void checkSteerLimits() {
  for (int i = 0; i < 4; i++) {
    noInterrupts();
    int32_t currentTicks = steer_ticks[i];
    interrupts();

    if (currentTicks < MIN_STEER_TICKS) {
      Serial.print("Warning: Motor ");
      Serial.print(i);
      Serial.print(" tick value (");
      Serial.print(currentTicks);
      Serial.println(") below lower limit.");
      
      // Save the safe value (lower limit) for recovery.
      savedSteerTicks[i] = MIN_STEER_TICKS;
      
      // Stop the motor by writing 0 to its PWM output.
      analogWrite(motors[i].pwm_pin, 0);
      motors[i].moving = false;
    }
    else if (currentTicks > MAX_STEER_TICKS) {
      Serial.print("Warning: Motor ");
      Serial.print(i);
      Serial.print(" tick value (");
      Serial.print(currentTicks);
      Serial.println(") above upper limit.");
      
      // Save the safe value (upper limit) for recovery.
      savedSteerTicks[i] = MAX_STEER_TICKS;
      
      // Stop the motor.
      analogWrite(motors[i].pwm_pin, 0);
      motors[i].moving = false;
    }
  }
}

// --------------------------------------------------------------------
// Function: recoverSteerPosition
// If the global variable 'busy' is true, this function commands each steering motor
// to "go back" by the difference (in encoder ticks) between its saved (safe) tick value 
// and its current tick value.
// After recovery, busy is reset to false.
// --------------------------------------------------------------------
void recoverSteerPosition() {
  if (!busy)
    return;
  
  Serial.println("Recovering steering positions...");
  
  for (int i = 0; i < 4; i++) {
    noInterrupts();
    int32_t current = steer_ticks[i];
    interrupts();
    
    // Calculate how many ticks to recover (move in the reverse direction).
    int32_t delta = -savedSteerTicks[i];
    if (abs(delta) >= TICK_DEADBAND) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" recovering by delta: ");
      Serial.println(delta);
      moveBy(delta, motors[i].pwm_pin, motors[i].dir_pin, motors[i].ticks, motors[i]);
    }
    else {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println(" recovery not needed (within deadband).");
    }
  }
  
  // Once recovery commands are issued, reset the busy flag.
  busy = false;
}

// --- Loop ---
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    //computeOdometry();
    publish_ticks();
 //   if (busy==LOW)
   // updateSteering(0);
     // Regularly check if any steering motor's encoder tick is out of bounds.

  // If (for example) a limit was exceeded and busy has been set true,
  // call recoverSteerPosition() to move motors back to their safe (saved) tick values.
  recoverSteerPosition();


}
