#include <Adafruit_TCS34725.h>
#include <Wire.h>

#define IR0 22
#define IR1 A0//11
#define IR2 A1//12
#define IR3 A2//13
#define IR4 A3//2
#define IR5 A4//3
#define IR6 A5//4
#define IR7 23

#define soundSensorPin A9

// #define SW1 47
// #define SW2 46
// #define SW3 47
// #define SW4 48
// #define SW5 49
#define SW6 47
// #define SW7 51

int task = 1;

#define ENA 2//5  //RIGHT
#define motorInput1 3//7 
#define motorInput2 4//8
#define motorInput3 6//9
#define motorInput4 5//10
#define ENB 7//6

//Ultrasonic 
#define trig0 10 // workin
#define echo0 9 //workin
#define trig1 12 //
#define echo1 11 //
#define trig2 35
#define echo2 34 //workin
#define trig3 27
#define echo3 26

#define MAX_SPEED 255
#define MAX_DISTANCE 25

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

int MotorBasespeed1 = 100; 

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {0, -15, -10, -5, 5, 10, 15, 0};

int directionArray[5] = {-1,-1,-1,-1};
int right[5] = {1,0,0,1};
int left[5] = {0,1,1,0};

int RMotor = 0;
int LMotor = 0;
int speedAdjust = 0;
int baseSpeed = 85; //final = 75

int count1 = 1;
int i = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 12;
float Kd = 0.1;//10
float Ki = 0;

unsigned long start_time; 
float max_distance = 10;
int t1;
int t2;

float sensor1 = 0;
float sensor2 = 0;
float sensor3 = 0;
float sensor4 = 0;

float frontLeftDistance;
float frontRightDistance;
float rightDistance;
float leftDistance;

int count = 0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(soundSensorPin, INPUT);

  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  pinMode(trig0, OUTPUT);
  pinMode(echo0, INPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);

  start_time = millis();

  set_forward();
  delay(200);
  task = 1;
}

int countU = 1;

void loop() {
  task=checkPointCounter();//Need more accurate ways
  taskSwitcher();

  switch (task) {
      case 1:
      
        task_01();
        //task 1 here
        break;
      case 2:
        //task_02();
        //task 2 here
        break;
      case 3:
        //task_03();
        //task 3 here
        break;
      case 4:
        //task_04();
       //task 4 here
        break;
      case 5:
        //task 5 here
        break;
      case 6:
        task_06();
        break;
      case 7:
        task_07();
        break;
  }
}

void taskSwitcher(){
  // if(digitalRead(SW1)==0){
  //   task=1;
  // }
  // else if(digitalRead(SW2)==0){
  //   task=2;
  // }
  // else if(digitalRead(SW3)==0){
  //   task=3;
  // }
  // else if(digitalRead(SW4)==0){
  //   task=4;
  // }
  // else if(digitalRead(SW5)==0){
  //   task=5;
  // }
  if(digitalRead(SW6)==0){
    task=6;
  }
  // else if(digitalRead(SW7)==0){
  //   task=7;
  // }
  return ;
}

void task_01(){
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
    analogWrite(ENA, RMotor);
    analogWrite(ENB, LMotor);
    set_forward();
    delay(50);
    stop();
    task = 2;
  }
  set_forward();
  PID_control();
  set_speed();
}

void task_02(){
  sensor1 = readUltrasonic(trig0, echo0);
  sensor2 = readUltrasonic(trig1, echo1);
  sensor3 = readUltrasonic(trig2, echo2);
  sensor4 = readUltrasonic(trig3, echo3);

  if (sensor1<0){sensor1=0;}
  if (sensor2<0){sensor2=0;}
  if (sensor3<0){sensor3=0;}
  if (sensor4<0){sensor4=0;}

  Serial.print(sensor1);
   Serial.print(" ");
    Serial.print(sensor2);
    Serial.print(" ");
    Serial.print(sensor3);
   Serial.print(" ");
    Serial.print(sensor4);
    Serial.println(" ");

    if(( sensor2<15 || sensor3<15) && (sensor2!=0) && (sensor3!=0)) {
      if (countU == 1){
      turnLeft();
      delay(4500);
      moveForward();
      delay(1000);
      countU = 2;
      }
      else if (countU == 2){
      turnRight();
      delay(4500);
      moveForward();
      delay(1000);
      // turnRight();
      // delay(500);
      countU = 3;
      }
    }
    set_forward();
    lineFollow();
}

void task_06(){
  int sensorData = digitalRead(soundSensorPin);

  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
    moveForward();
    delay(500);
    stop();
    task = 7;
  }
   if (sensorData == LOW) {
    set_forward();
    lineFollow();
    if(IR_val[1]==0 && IR_val[2]==0 &&  IR_val[5]==1 && IR_val[6]==1){
      stop();
      turnLeft();
       lineFollow();
    }
    if(IR_val[1]==1 && IR_val[2]==1 &&  IR_val[5]==0 && IR_val[6]==0){
      stop();
      turnRight();
      lineFollow();
    }
  } 
  else if (sensorData == HIGH){
    stop();
  }
}

void task_07(){
  float dist1 = readUltrasonic(trig1,echo1);
  float dist2 = readUltrasonic(trig2,echo2);
//Serial.println(dist1);
//Serial.println(dist2);
 if (dist1 < max_distance) {
    t1 = millis();
  }
  else if (dist2 < max_distance) {
    t2 = millis();
  }
  if (t1 > 0 && t2 > 0) {       // if both sensors have nonzero timestamps
    if (t1 < t2) {                      // if left sensor triggered first
      //Serial.println("Left to right");    // direction is left to right
      //stop();
    }
    else if (t2 < t1) {  
      //Serial.println("Right to left");
      while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0)){
        lineFollow();
      }          
      // if right sensor triggered first
        turnRight();
        delay(1000);
        while (!(IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0)){
          lineFollow();
      }
      analogWrite(ENA, 75);
      analogWrite(ENB, 75);
      set_forward();
      delay(500);
      stop();
      task = 1;
    }
  // else{
  //   //Serial.println(" ");
  // }
  t1=0;
  t2=0;
}
}

void task_04(){
 //comment
}

int colorIdentification(){
  int color;
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (r>b){
    color= 1 ;//1 for red
  }
  else if (b<r){
    color= 0 ;//0 for blue
  }
  return color; 
}

void lineFollow(){
  read_IR();
  // if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
  //   //analogWrite(ENB, 50);
  //   //delay(100);
  //   stop();
  // }
  PID_control();
  set_speed();
}


float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  if (duration>0){
    return distance;
  }
}
void turnLeft(){
  analogWrite(ENA, 80);
  analogWrite(ENB, 0);

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
   //adjust the delay
  Serial.println("Lturn");
}

void turnRight(){
  analogWrite(ENA, 50);
  analogWrite(ENB, 80);

  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
 //adjust the delay
  Serial.println("Rturn");
}

//Move with a given velocity
void moveForward() {
  // Move the robot forward
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  
}

void moveBackward() {
  analogWrite(ENA, 75);
  analogWrite(ENB, 75);

  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

void reverse(){
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
}
void PID_control() {
  error = 0;

  for (int i = 0; i < 6; i++)
  {
    error += IR_weights[i+1]*IR_val[i+1];
  }
  P = error;
  I = I + error; 
  D = error - previousError;

  previousError = error;

  speedAdjust = (Kp*P + Ki*I + Kd*D);;

  RMotor = baseSpeed + speedAdjust;
  LMotor = baseSpeed - speedAdjust;

  if (IR_val[1]==1 && IR_val[6]==1){
    if (RMotor<10){
      RMotor = 10;
  }
    if (LMotor<10){
        LMotor = 10;
    }
  }
  else {
  if (RMotor<0){
    analogWrite(ENA, 50);
    analogWrite(ENB, LMotor);

    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
  }
  if (LMotor<0){
    analogWrite(ENA, RMotor);
    analogWrite(ENB, 50);

    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, HIGH);
  }
  }
  if (LMotor>MAX_SPEED){
    LMotor = MAX_SPEED;
  }
  if (RMotor>MAX_SPEED){
    RMotor = MAX_SPEED;
  }
  Serial.print(LMotor);
  Serial.print(" ");
  Serial.println(RMotor);

}

void read_IR(){
  IR_val[0] = digitalRead(IR0);
  IR_val[1] = digitalRead(IR1);
  IR_val[2] = digitalRead(IR2);
  IR_val[3] = digitalRead(IR3);
  IR_val[4] = digitalRead(IR4);
  IR_val[5] = digitalRead(IR5);
  IR_val[6] = digitalRead(IR6);
  IR_val[7] = digitalRead(IR7);
}

int checkPointCounter(){
  read_IR();
  int summ=0;
  for ( int i = 0; i < 8; i++ )
    summ += IR_val[ i ];
  if (summ==0){
    count+=1;
  }
  return;
}

void set_speed(){
  analogWrite(ENA, RMotor);
  analogWrite(ENB, LMotor);
}

//Move with PID Controlled velocity
void set_forward(){
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void stop(){
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
