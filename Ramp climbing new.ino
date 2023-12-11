#include <MPU6050.h>
#include <math.h>
#include <Wire.h>

MPU6050 mpu;
const float gravity = 9.81; // acceleration due to gravity in m/s^2
const float wheel_radius = 32.5/1000; // radius of the wheel in meters
const float robot_mass = 1.0; // mass of the robot in kilograms
const float max_speed = 0.3;//max speed of robot in ms^-1

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4G);
  
}
float torque(double ang){
  double torq = (max_speed+ sin(ang)*gravity)/2*wheel_radius*robot_mass;
  return torq;

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
  Serial.print("torque: ");
  Serial.println(torque(tiltAngleRad));

  delay(1000);
}

