#include <Servo.h>

// Define ultrasonic sensor pins
#define FRONTL_SENSOR_TRIG 2
#define FRONTL_SENSOR_ECHO 3
#define LEFT_SENSOR_TRIG 4
#define LEFT_SENSOR_ECHO 11
#define RIGHT_SENSOR_TRIG 12
#define RIGHT_SENSOR_ECHO 13
#define FRONTR_SENSOR_TRIG 9
#define FRONTR_SENSOR_ECHO 10

// Define motor control pins
#define MOTOR_A_EN 6
#define MOTOR_B_EN 5
#define MOTOR_A_IN1 22
#define MOTOR_A_IN2 23
#define MOTOR_B_IN1 24
#define MOTOR_B_IN2 25

long leftDistance;

// Define obstacle detection thresholds
int frontObstacleThreshold = 10; // Adjust this value based on your sensors and robot
int leftObstacleThreshold = 10;  // Adjust this value based on your sensors and robot

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(FRONTL_SENSOR_TRIG, OUTPUT);
  pinMode(FRONTL_SENSOR_ECHO, INPUT);
  pinMode(FRONTR_SENSOR_TRIG, OUTPUT);
  pinMode(FRONTR_SENSOR_ECHO, INPUT);
  pinMode(LEFT_SENSOR_TRIG, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIG, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);

  // Initialize motor control pins
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

}

long frontLeftDistance;
long frontRightDistance;
long rightDistance;



long measureDistance(int trigPin, int echoPin) {
  // Measure the distance using the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return (duration / 2) / 29.1; // Convert the duration to centimeters
}

void moveForward() {
  // Move the robot forward
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_A_EN, 50);
  analogWrite(MOTOR_B_EN, 50);
}

void moveBackward() {
  // Move the robot forward
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_A_EN, 50);
  analogWrite(MOTOR_B_EN, 50);
}

void stop() {
  // Stop the robot
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  //delay(1000);
}

void turnRight() {
  // Turn the robot LEFt
  
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  analogWrite(MOTOR_A_EN, 85);
  analogWrite(MOTOR_B_EN,65);
}

void turnLeft() {
  // Turn the robot LEFT
  
  digitalWrite(MOTOR_B_IN1,LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_EN, 85);
  analogWrite(MOTOR_B_EN,65);
}

void loop() {
  // put your main code here, to run repeatedly:
  frontLeftDistance = measureDistance(FRONTL_SENSOR_TRIG, FRONTL_SENSOR_ECHO);
  frontRightDistance = measureDistance(FRONTR_SENSOR_TRIG, FRONTR_SENSOR_ECHO);
  leftDistance = measureDistance(LEFT_SENSOR_TRIG, LEFT_SENSOR_ECHO);
  rightDistance = measureDistance(RIGHT_SENSOR_TRIG, RIGHT_SENSOR_ECHO);

  Serial.print(frontLeftDistance);
  Serial.print(" "); 
  Serial.print(frontRightDistance);
  Serial.print(" "); 
  Serial.print(rightDistance); 
  Serial.print(" ");
  Serial.print(leftDistance);
  Serial.println();

  if ((frontLeftDistance < frontObstacleThreshold) || (frontRightDistance < frontObstacleThreshold)) {
    if (frontLeftDistance > frontRightDistance) {
      stop();
      delay(200);
      turnLeft();
      delay(200);
      stop();
      delay(200); 
    }
    else {
      stop();
      delay(200);
      turnRight();
      delay(200);
      stop();
      delay(200);
    }
  }
  
  if ((leftDistance < leftObstacleThreshold) || (rightDistance < leftObstacleThreshold)){
    if(leftDistance < leftObstacleThreshold){
      stop();
      delay(200);
      turnRight();
      delay(200);
      stop();
      delay(200);
    }
    else{
      stop();
      delay(200);
      turnLeft();
      delay(200);
      stop();
      delay(200);
    }
    
  }

  
    moveForward();
    // delay(500);
    // stop();
    // delay(500);
}

