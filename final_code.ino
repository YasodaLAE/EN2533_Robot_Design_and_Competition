#include <Adafruit_TCS34725.h>
#include <Wire.h>

#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5

#define SW1 45
#define SW2 46
#define SW3 47
#define SW4 48
#define SW5 49
#define SW6 50
#define SW7 51

int task;

#define ENA 4
#define motorInput1 35
#define motorInput2 36
#define motorInput3 38
#define motorInput4 39
#define ENB 5

#define MAX_SPEED 150

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

int MotorBasespeed1 = 100; 

int IR_val[] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-20, -10, -5, 5, 10, 20};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 5.5;
float Kd = 0;//10
float Ki = 0;

int count =0;

void PID_control();
void read_IR();
void set_speed();
void set_forward();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  set_forward();
  delay(2000);

}

void loop() {
  task=checkPointCounter();//Need more accurate ways
  taskSwitcher();

  switch (task) {
      case 'a':
        task_01();
        //task 1 here
        break;
      case 'b':
        //task_02();
        //task 2 here
        break;
      case 'c':
        //task_03();
        //task 3 here
        break;
      case 'd':
        task_04();
       //task 4 here
        break;
      case 'e':
        //task 5 here
        break;
      case 'f':
        //task 6 here
        break;
      case 'g':
       //task 7 here
        break;

  
  }
}

void taskSwitcher(){
  if(digitalRead(SW1)==1){
    task='a';
  }
  else if(digitalRead(SW2)==1){
    task='b';
  }
  else if(digitalRead(SW3)==1){
    task='c';
  }
  else if(digitalRead(SW4)==1){
    task='d';
  }
  else if(digitalRead(SW5)==1){
    task='e';
  }
  else if(digitalRead(SW6)==1){
    task='f';
  }
  else if(digitalRead(SW6)==1){
    task='g';
  }
  return ;

}

void task_01(){
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0){
    stop();
    //while (1){}
  }
  else{
  PID_control();
  set_speed();
  }
}
void task_04(){

  //Drag the box 
  int color =colorIdentification(); //Identify the color of box
  //Reverse till the checkpoint
  read_IR();
  int summ=sum();
  if (summ<2){
    stop();
  }

  if (color==0){ //Should combine the IR effect also
    TurnLeft();
  }
  else{
    TurnRight();
  }

  int summ = sum();
  read_IR();
  if (IR_val[0]==1 && IR_val[1]==1 && IR_val[2]==1 && IR_val[3]==1 && IR_val[4]==1 && IR_val[5]==1){
    turnLeft();
  }
  else if (summ<2){
    stop();
    delay(500);
    turnLeft();
  }
  else if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[4]==1 && IR_val[5]==1){
    stop();
    delay(500);
    turnLeft();
  }

  else if (IR_val[0]==1 && IR_val[1]==1 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0){
    stop();
    delay(500);
    turnRight();
  }
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0){
    stop();
    moveForward();
    delay(500);

  }
  else{
  PID_control();
  set_speed();
  }
  //should keep the box on the colored square
}
int checkPointCounter(){
  read_IR();
  int summ=sum();
  if (summ<2){
    count+=1;
  }
  return count;
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

void turnLeft(){
  digitalWrite(motorInput1,LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  analogWrite(ENA, 85);
  analogWrite(ENB,65);
}

void turnRight() {
  
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  analogWrite(ENA, 85);
  analogWrite(ENB,65);
}

//Move with a given velocity
void moveForward() {
  // Move the robot forward
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
}

void reverse(){
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
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

  speedAdjust = (Kp*P + Ki*I + Kd*D);;

  LMotorSpeed = 120 - speedAdjust;
  RMotorSpeed = 105 + speedAdjust;

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
}

void set_speed(){
  analogWrite(ENA, LMotorSpeed);
  analogWrite(ENB, RMotorSpeed);
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

int sum(){
  int summ;
  for ( int i = 0; i < 6; i++ )
    summ += IR_val[ i ];
  return summ;
}