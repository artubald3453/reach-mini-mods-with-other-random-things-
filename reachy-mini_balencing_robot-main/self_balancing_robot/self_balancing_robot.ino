/*
 * Self-Balancing Two-Wheel Robot with Serial Control
 * 
 * This robot uses PID control and an MPU6050 IMU to maintain balance
 * Commands via USB Serial: 'f' (forward), 'b' (backward), 'l' (left), 'r' (right)
 * 
 * Hardware Requirements:
 * - Arduino Nano/Uno
 * - MPU6050 Gyroscope/Accelerometer
 * - L298N Motor Driver
 * - 2x DC Motors with encoders (recommended)
 * - 2x Wheels
 * - Battery (7.4V-12V LiPo recommended)
 */

#include <Wire.h>
#include <MPU6050.h>

// ==================== PIN DEFINITIONS ====================
// Motor A (Left)
#define MOTOR_A_IN1 7
#define MOTOR_A_IN2 8
#define MOTOR_A_PWM 9  // Must be PWM pin

// Motor B (Right)
#define MOTOR_B_IN1 4
#define MOTOR_B_IN2 5
#define MOTOR_B_PWM 10 // Must be PWM pin

// ==================== GLOBAL VARIABLES ====================
MPU6050 mpu;

// PID Variables for balancing
float Kp = 40.0;   // Proportional gain (tune this first)
float Ki = 0.8;    // Integral gain (tune second)
float Kd = 1.2;    // Derivative gain (tune third)

float targetAngle = -2.0;  // Target angle (adjust based on robot's center of gravity)
float currentAngle = 0;
float angleError = 0;
float lastError = 0;
float errorSum = 0;
float pidOutput = 0;

// Motor speed variables
int baseSpeed = 0;      // Base forward/backward speed from commands
int turnSpeed = 0;      // Turning speed from left/right commands
int motorSpeedA = 0;    // Final speed for left motor
int motorSpeedB = 0;    // Final speed for right motor

// Command variables
char command = 's';     // Current command: f/b/l/r/s(stop)
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 500; // Stop if no command for 500ms

// Timing
unsigned long lastTime = 0;
float dt = 0.01; // 10ms loop time

// Complementary filter variables
float accAngle = 0;
float gyroRate = 0;
float filteredAngle = 0;
const float ALPHA = 0.98; // Complementary filter constant

// Safety limits
const int MAX_SPEED = 255;
const int MAX_ANGLE = 45;  // Maximum tilt angle before emergency stop
const float MAX_ERROR_SUM = 100.0; // Anti-windup for integral term

// ==================== SETUP ====================
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Self-Balancing Robot Initializing...");
  
  // Initialize motor pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  
  // Initialize I2C and MPU6050
  Wire.begin();
  mpu.initialize();
  
  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1); // Halt if sensor not found
  }
  
  // Calibrate MPU6050
  Serial.println("Calibrating MPU6050... Keep robot upright and still!");
  delay(2000);
  calibrateMPU6050();
  
  Serial.println("Calibration complete!");
  Serial.println("Commands: f=forward, b=backward, l=left, r=right, s=stop");
  Serial.println("Robot ready!");
  
  lastTime = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Read sensor data and calculate angle
  readSensorAndCalculateAngle();
  
  // Check for emergency stop condition
  if (abs(currentAngle) > MAX_ANGLE) {
    emergencyStop();
    return;
  }
  
  // Read serial commands
  readSerialCommands();
  
  // Calculate PID output for balancing
  calculatePID();
  
  // Apply command speeds (forward/backward/turn)
  applyCommandSpeeds();
  
  // Calculate final motor speeds
  motorSpeedA = constrain(pidOutput + baseSpeed - turnSpeed, -MAX_SPEED, MAX_SPEED);
  motorSpeedB = constrain(pidOutput + baseSpeed + turnSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Drive motors
  driveMotors(motorSpeedA, motorSpeedB);
  
  // Debug output (comment out for better performance)
  if (currentTime % 100 < dt * 1000) { // Print every ~100ms
    Serial.print("Angle: "); Serial.print(currentAngle);
    Serial.print(" | PID: "); Serial.print(pidOutput);
    Serial.print(" | SpeedA: "); Serial.print(motorSpeedA);
    Serial.print(" | SpeedB: "); Serial.println(motorSpeedB);
  }
  
  // Maintain consistent loop time
  delay(10); // 10ms = 100Hz update rate
}

// ==================== SENSOR READING ====================
void readSensorAndCalculateAngle() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Read raw sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate angle from accelerometer (in degrees)
  // For a two-wheel robot, we care about pitch (rotation around Y-axis)
  accAngle = atan2(ax, az) * 180.0 / PI;
  
  // Calculate gyro rate (convert to degrees/second)
  gyroRate = gy / 131.0; // For default ±250°/s range
  
  // Complementary filter to combine accelerometer and gyroscope
  filteredAngle = ALPHA * (filteredAngle + gyroRate * dt) + (1.0 - ALPHA) * accAngle;
  
  currentAngle = filteredAngle;
}

// ==================== PID CONTROL ====================
void calculatePID() {
  // Calculate error
  angleError = currentAngle - targetAngle;
  
  // Proportional term
  float P = Kp * angleError;
  
  // Integral term (with anti-windup)
  errorSum += angleError * dt;
  errorSum = constrain(errorSum, -MAX_ERROR_SUM, MAX_ERROR_SUM);
  float I = Ki * errorSum;
  
  // Derivative term
  float D = Kd * (angleError - lastError) / dt;
  
  // Calculate total PID output
  pidOutput = P + I + D;
  
  // Store error for next iteration
  lastError = angleError;
}

// ==================== SERIAL COMMANDS ====================
void readSerialCommands() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    
    // Only accept valid commands
    if (incoming == 'f' || incoming == 'b' || incoming == 'l' || 
        incoming == 'r' || incoming == 's') {
      command = incoming;
      lastCommandTime = millis();
    }
  }
  
  // Timeout check - stop if no command received recently
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    command = 's';
  }
}

// ==================== COMMAND SPEEDS ====================
void applyCommandSpeeds() {
  const int DRIVE_SPEED = 30;  // Base driving speed
  const int TURN_SPEED = 25;   // Turning speed
  
  switch (command) {
    case 'f': // Forward
      baseSpeed = DRIVE_SPEED;
      turnSpeed = 0;
      break;
      
    case 'b': // Backward
      baseSpeed = -DRIVE_SPEED;
      turnSpeed = 0;
      break;
      
    case 'l': // Left (rotate in place)
      baseSpeed = 0;
      turnSpeed = TURN_SPEED;
      break;
      
    case 'r': // Right (rotate in place)
      baseSpeed = 0;
      turnSpeed = -TURN_SPEED;
      break;
      
    case 's': // Stop
    default:
      baseSpeed = 0;
      turnSpeed = 0;
      break;
  }
}

// ==================== MOTOR CONTROL ====================
void driveMotors(int speedA, int speedB) {
  // Motor A (Left)
  if (speedA > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
  }
  analogWrite(MOTOR_A_PWM, abs(speedA));
  
  // Motor B (Right)
  if (speedB > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
  } else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
  }
  analogWrite(MOTOR_B_PWM, abs(speedB));
}

// ==================== CALIBRATION ====================
void calibrateMPU6050() {
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    
    delay(2);
  }
  
  // Calculate offsets
  int16_t ax_offset = -(ax_sum / samples);
  int16_t ay_offset = -(ay_sum / samples);
  int16_t az_offset = 16384 - (az_sum / samples); // 1g = 16384 in default config
  int16_t gx_offset = -(gx_sum / samples);
  int16_t gy_offset = -(gy_sum / samples);
  int16_t gz_offset = -(gz_sum / samples);
  
  // Apply offsets
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  
  Serial.println("Offsets calculated and applied");
}

// ==================== EMERGENCY STOP ====================
void emergencyStop() {
  // Stop all motors
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  
  // Reset integral term
  errorSum = 0;
  
  Serial.println("EMERGENCY STOP - Angle too large!");
  delay(1000);
}

