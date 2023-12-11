#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

Servo myServoUp;//Top Servo
Servo myServoDown;//Bottom Servo

int posA =0 ;
int posB =0 ;

const int trig5=48;//48,49
const int echo5=49;
long duration;
float distance;

#define IR0 52
#define IR1 A0//11
#define IR2 A1//12
#define IR3 A2//13
#define IR4 A3//2
#define IR5 A4//3
#define IR6 A5//4
#define IR7 53

//#define soundSensorPin A7

#define ENA 2//5  //RIGHT
#define motorInput1 3//7 
#define motorInput2 4//8
#define motorInput3 6//9
#define motorInput4 5//10
#define ENB 7//6

#define MAX_SPEED 255
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {0, -15, -10, -5, 5, 10, 15, 0};

int RMotor = 0;
int LMotor = 0;
int speedAdjust = 0;
int baseSpeed = 85; //final = 75
// unsigned long lastEvent = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 12;//11 ok 12th ok 10 not ok
float Kd = 0.1;//10, 1 final = 6 //7,8,9 THIBBE
float Ki = 0; //final = 0

void PID_control();
void read_IR();
void set_speed();
void set_forward();


void setup() {

  pinMode(trig5,OUTPUT);
  pinMode(echo5,INPUT);

  myServoUp.attach(10); 
  myServoDown.attach(9); 
 
  Serial.begin(9600);

  //pinMode(soundSensorPin, INPUT);

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

  // analogWrite(ENA, 0);
  // analogWrite(ENB, 255);
  set_forward();
  delay(200);

}



void lineFollow() {
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
    //analogWrite(ENB, 50);
    //delay(100);
    stop();
    while (1){

    }
  }
  set_forward();
  PID_control();
  set_speed();
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
  
  Serial.print(IR_val[1]);
  Serial.print(IR_val[2]);
  Serial.print(IR_val[3]);
  Serial.print(IR_val[4]);
  Serial.print(IR_val[5]);
  Serial.println(IR_val[6]);
}

void set_speed(){
  analogWrite(ENA, RMotor);
  analogWrite(ENB, LMotor);
}

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

float readUltraSonic(int trigPin, int echoPin){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}

void pickBox(){
  myServoUp.write(170);
  delay(500);
  myServoDown.write(0);
  delay(500);
  myServoUp.write(0);
  delay(500);
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
void turnBack(){ //U turn
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  delay(1000);
}

void turnLeft(){
  analogWrite(ENA, 80);
  analogWrite(ENB, 50);

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  delay(500);
}

void turnRight(){
  analogWrite(ENA, 50);
  analogWrite(ENB, 80);

  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  delay(500);
}

void detectT_blue(){
  if (IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0){
    turnRight();
  }
}

void detectT_red(){
  if (IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0){
    turnLeft();
  }
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;

  if(13>distance ){
  distance=readUltraSonic(trig5, echo5);  
    lineFollow();
  }
  else if(5<distance && distance<13){
    stop();
    readUltraSonic(trig5, echo5);
    set_forward();
    analogWrite(ENA, 60);
    analogWrite(ENB, 60);
    delay(50);

  }
  else
  {
    pickBox();
    colorIdentification();
    turnBack();
    lineFollow();
    detectT_blue();
    detectT_red();
    mazeSolve();
    
  }
  
}
