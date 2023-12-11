https://tronic.lk/product/mpu-6050-triple-axis-analog-accelerometer-gyroscope-mod
/* getting data from the accelerometer*/
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  Serial.print("AccelX: ");
  Serial.print(ax);
  Serial.print("  AccelY: ");
  Serial.print(ay);
  Serial.print("  AccelZ: ");
  Serial.println(az);

  delay(1000);
}
**************************************************************
/*method of calculating angle */

#include <MPU6050.h>
#include <math.h>
#include <Wire.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4G);
  
}

void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float accelX = ax / 16384.0*9.81;
  float accelY = ay / 16384.0*9.81;
  float accelZ = az / 16384.0*9.81;
  
  double totalAccel = sqrt(pow(accelX,2)+pow(accelY,2) + pow(accelZ,2));

// Calculate tilt angle in radians
  double tiltAngleRad = acos(accelZ / totalAccel);

// Convert tilt angle to degrees
  double tiltAngleDeg = tiltAngleRad * (180.0 / PI);

// Print or use the tilt angle in degrees
  
  Serial.print("Tilt Angle (degrees): ");
  Serial.println(tiltAngleDeg);
  delay(1000);
}

*****************************************************************
/* calculating torque required*/

const float gravity = 9.81; // acceleration due to gravity in m/s^2
const float wheel_radius = 0.05; // radius of the wheel in meters
const float robot_mass = 5.0; // mass of the robot in kilograms
const float incline_angle = 30.0; // angle of the ramp in degrees

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Calculate the torque required to climb the ramp
  float incline_angle_rad = radians(incline_angle);
  float gravitational_force = robot_mass * gravity;
  float torque_required = gravitational_force * wheel_radius * sin(incline_angle_rad);

  // Print the calculated torque
  Serial.print("Torque Required: ");
  Serial.print(torque_required, 2);
  Serial.println(" Nm");

  delay(1000); // Delay for readability
}
*****************************************************************
/* controlling speed of the motor to get the required torque*/

// Motor driver control pins
const int motorEnablePin = 10; // PWM speed control pin
const int motorDirectionPin = 9; // Direction control pin

// Torque and speed control variables
float targetTorque = 5.0; // Replace with the calculated torque
int motorSpeed;

void setup() {
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorDirectionPin, OUTPUT);
  analogWrite(motorEnablePin, 0); // Initialize motor speed to zero
}

void loop() {
  // Calculate motor speed based on torque (adjust this calculation based on your motor's characteristics)
  motorSpeed = map(targetTorque, 0, 10, 0, 255); // Example mapping

  // Set motor direction (adjust based on your desired setup)
  digitalWrite(motorDirectionPin, HIGH); // Example: clockwise rotation
  
  // Set motor speed
  analogWrite(motorEnablePin, motorSpeed);

  delay(1000); // Delay for readability
}

