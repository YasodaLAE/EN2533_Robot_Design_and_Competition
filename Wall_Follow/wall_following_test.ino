#include "NewPing.h"

#define TRIGGER_PIN_1 9
#define ECHO_PIN_1 10

#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 10

#define TRIGGER_PIN_3 9
#define ECHO_PIN_3 10
// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 400

// NewPing setup of pins and maximum distance.
NewPing sonarFront(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonarLeft(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);

//number of reading to take average distance
int iterations = 5;
int baseSpeed = 120;

float distanceFront, distanceRight, distanceLeft, pidSum;

double proportional, integral, derivative;

const double desiredState = (double) 10;
const double kp = 4;
const double ki = 0.5;
const double kd = 2;

double totalError = 0.0;
double prevError = 0.0;
double curError = 0.0;

String direction;

// Motor A connections (Right motor)
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections (Left motor)
int enB = 3;
int in3 = 5;
int in4 = 4;

void setup() {
  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void loop() {
  distanceFront = (sonarFront.ping_median(iterations) / 2) * 0.0343;
  distanceRight = (sonarRight.ping_median(iterations) / 2) * 0.0343;
  distanceLeft = (sonarLeft.ping_median(iterations) / 2) * 0.0343;
  
  //Kp*e(t)
  if ((distanceRight > 10) && (distanceLeft < 10)){
    curError = desiredState - distanceLeft;
    direction = "Left"
  }
  else {
    curError = desiredState - distanceRight;
    direction = "Right"
  }
  proportional = kp*curError;

  //Ki*int(e(t))
  totalError += curError;
  integral = ki*totalError;

  //Kd*d(e(t))
  derivative = kd*(curError - prevError);
  prevError = curError; 

  pidSum = proportional + integral + derivative;

  speedControl(pidSum, direction);
}

void speedControl(float pidTotal, String dir){
  digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);

  if (dir == "Right"){
    analogWrite(enA, baseSpeed - pidTotal);
    analogWrite(enB, baseSpeed + pidTotal);
  }
  else {
    analogWrite(enA, baseSpeed + pidTotal);
    analogWrite(enB, baseSpeed - pidTotal);
  }
}
