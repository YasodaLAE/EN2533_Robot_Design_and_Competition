#include <NewPing.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2,3); //(Rx,Tx)

#define TRIGGER_PIN_1 4
#define ECHO_PIN_1 5

#define TRIGGER_PIN_2 A0
#define ECHO_PIN_2 A1

#define TRIGGER_PIN_3 10
#define ECHO_PIN_3 11
// Maximum distance we want to ping for (in centimeters).
#define MAX_DISTANCE 100

#define ledPin1 8
#define ledPin2 9
#define ledPin3 12

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
double kp = 4;
double ki = 0.5;
double kd = 2;

double totalError = 0.0;
double prevError = 0.0;
double curError = 0.0;

// Motor A connections (Right motor)
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections (Left motor)
int enB = 3;
int in3 = 5;
int in4 = 4;

double tp = 0, ti = 0, td = 0;

void setup() {
  //bluetooth
  bluetooth.begin(9600);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
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
  
  if (bluetooth.available()){
    char c = bluetooth.read();
    if(c=='p'){
      float i = 10;
      while (bluetooth.available()){
        char d = bluetooth.read();
        if (d!='.'){
        tp += (d-'0')*i;
        i = i/10;}
        digitalWrite(ledPin1, HIGH);
        delay(700);
        digitalWrite(ledPin1, LOW);
      }
    }
    else if(c=='i'){
      float i = 10;
      while (bluetooth.available()){
        char d = bluetooth.read();
        if (d!='.'){
        ti += (d-'0')*i;
        i = i/10;}
        digitalWrite(ledPin2, HIGH);
        delay(700);
        digitalWrite(ledPin2, LOW);
      }
      ki = ti;
    }
    else if(c=='d'){
      float i = 10;
      while (bluetooth.available()){
        char d = bluetooth.read();
        if (d!='.'){
        td += (d-'0')*i;
        i = i/10;}
        digitalWrite(ledPin3, HIGH);
        delay(700);
        digitalWrite(ledPin3, LOW);
      }
      kd = td;
    }
  }

  //Kp*e(t)
  curError = desiredState - distanceRight;
  proportional = kp*curError;

  //Ki*int(e(t))
  totalError += curError;
  integral = ki*totalError;

  //Kd*d(e(t))
  derivative = kd*(curError - prevError);
  prevError = curError; 

  pidSum = proportional + integral + derivative;
}

void speedControl(float pidTotal){
  digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);

  analogWrite(enA, baseSpeed - pidTotal);
  analogWrite(enB, baseSpeed + pidTotal);
}
