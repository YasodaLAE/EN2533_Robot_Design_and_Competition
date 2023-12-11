//Ir  0 for white line
//    1 for black lines
//Ir  0 for white line
//    1 for black lines
#include <SoftwareSerial.h>

SoftwareSerial bt(24,26);
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


#define IR1 A5    
#define IR2 A4
#define IR3 A3
#define IR4 A2  //LEFT
#define IR5 A1
#define IR6 A0



#define MAX_SPEED 240

int MotorBasespeed = 55; 

int IR_val[6] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-30 ,-20,-10, 10, 20, 30};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 0.4;//1.5-high 1.0 high 0.5
float Kd = 0;
float Ki = 0.0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

// Variables for sensor distances
long frontLeftDistance;
long frontRightDistance;
long rightDistance;
long leftDistance;

// Define obstacle detection thresholds
int frontObstacleThreshold = 10; // Adjust this value based on your sensors and robot
int leftObstacleThreshold = 10;  // Adjust this value based on your sensors and robot
// int rightDistance = 10; // Adjust this value based on your sensors and robot
// int leftDistance = 10;  // Adjust this value based on your sensors and robot

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //bt.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  // Initialize the ultrasonic sensor pins
  pinMode(FRONTL_SENSOR_TRIG, OUTPUT);
  pinMode(FRONTL_SENSOR_ECHO, INPUT);
  pinMode(FRONTR_SENSOR_TRIG, OUTPUT);
  pinMode(FRONTR_SENSOR_ECHO, INPUT);
  pinMode(LEFT_SENSOR_TRIG, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIG, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);

  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);

  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  set_forward();
  delay(1000);

}

void loop() {
  
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

  if((frontLeftDistance < frontObstacleThreshold) || (frontRightDistance < frontObstacleThreshold)||(leftDistance < leftObstacleThreshold) || (rightDistance < leftObstacleThreshold)){
    obstacle_check();  
  }
  else{
    read_IR();
    PID_control();
    set_speed();
  }

  
  // Serial.println(IR_val[0]);
  // if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 ){//
  //   stop();
  //   //   set_speed();
  //   //   //while (1){}
  // }
   // Measure distances using ultrasonic sensors
  
}

void PID_control() {
  error = 0;

  for (int i = 0; i < 6; i++)
  {
    error += IR_weights[i]*IR_val[i];
  }
  P = error;
  I = I + error; 
  D = error - previousError;

  previousError = error;

  speedAdjust = (Kp*P + Ki*I + Kd*D);
  bt.print("pid Value ");
  bt.print(speedAdjust);
  bt.println();
  // Serial.print("Speed Adjust ");
  // Serial.println(speedAdjust);

  LMotorSpeed = MotorBasespeed - speedAdjust;   //135
  RMotorSpeed = MotorBasespeed + speedAdjust;   //125


  if (LMotorSpeed<0){
    LMotorSpeed = 0;
  }
  if (RMotorSpeed<0){
    RMotorSpeed = 0;
  }
  if (RMotorSpeed>MAX_SPEED){
    RMotorSpeed = MAX_SPEED;
  }
  if (RMotorSpeed>MAX_SPEED){
    RMotorSpeed = MAX_SPEED;
  }
}

void read_IR(){
  IR_val[0] = digitalRead(IR1);
  IR_val[1] = digitalRead(IR2);
  IR_val[2] = digitalRead(IR3);
  IR_val[3] = digitalRead(IR4);
  IR_val[4] = digitalRead(IR5);
  IR_val[5] = digitalRead(IR6);
  // Serial.print(IR_val[0]);
  // Serial.print(IR_val[1]);
  // Serial.print(IR_val[2]);
  // Serial.print(IR_val[3]);
  // Serial.println(IR_val[4]);
  // if ((IR_val[0]==0) || (IR_val[1]==0) || (IR_val[2]==0) || (IR_val[3]==0) || (IR_val[4]==0) || (IR_val[5]==0)){
  
  // }

}

void set_speed(){
  analogWrite(MOTOR_A_EN, RMotorSpeed);
  analogWrite(MOTOR_B_EN, LMotorSpeed);
  // delay(50);
  // analogWrite(ENA, RMotorSpeed);
  // analogWrite(ENB, LMotorSpeed);
}

void set_forward(){
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void stop(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}



//For Obstacle avoidance

void obstacle_check(){
  if ((frontLeftDistance < frontObstacleThreshold) || (frontRightDistance < frontObstacleThreshold)) {
    if (frontLeftDistance > frontRightDistance) {
      stop();
      delay(500);
      turnLeft();
      delay(200);
      stop();
      delay(500); 
    }
    else {
      stop();
      delay(500);
      turnRight();
      delay(200);
      stop();
      delay(500);
    }
  }
  
  if ((leftDistance < leftObstacleThreshold) || (rightDistance < leftObstacleThreshold)){
    if(leftDistance < leftObstacleThreshold){
      stop();
      delay(500);
      turnRight();
      delay(200);
      stop();
      delay(500);
    }
    else{
      stop();
      delay(500);
      turnLeft();
      delay(200);
      stop();
      delay(500);
    }
    
  }

  
    moveForward();
    // delay(500);
    // stop();
    // delay(500);
}


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