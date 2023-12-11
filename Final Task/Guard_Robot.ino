#define IR1 11
#define IR2 12
#define IR3 13
#define IR4 2
#define IR5 3
#define IR6 4

#define ENA 5
#define motorInput1 7
#define motorInput2 8
#define motorInput3 9
#define motorInput4 10
#define ENB 6

//Ultrasonic pins
#define trig1 A4
#define echo1 A5
#define trig2 A2
#define echo2 A3

#define MAX_SPEED 150

int IR_val[] = {0, 0, 0, 0, 0, 0};
int IR_weights[6] = {-20, -10, -5, 5, 10, 20};
int directionArray[5] = {-1,-1,-1,-1};
int right[5] = {1,0,0,1};
int left[5] = {0,1,1,0};

int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;
int baseSpeed = 75; //final = 75
int count = 1;
int i = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 3.5;//{5.5-,1+,3++,4+} final = 3.5
float Kd = 6;//10, 1 final = 6
float Ki = 0; //final = 0
unsigned long start_time; 
float max_distance = 10;
int t1;int t2;

void PID_control();
void read_IR();
void set_speed();
void set_forward();
void linFollow();

void setup() {
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

  pinMode(trig1, OUTPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  start_time = millis();
}

void loop() {
  float dist1 = readUltrasonic(trig1,echo1);
  float dist2 = readUltrasonic(trig2,echo2);

 if (dist1 < max_distance) {
    t1 = millis();
  }
  else if (dist2 < max_distance) {
    t2 = millis();
  }
 
  if (t1 > 0 && t2 > 0) {       // if both sensors have nonzero timestamps
    if (t1 < t2) {                      // if left sensor triggered first
      // direction is left to right
    }
    else if (t2 < t1) {  
      set_forward();           // if right sensor triggered first
      lineFollow();
      // direction is right to left
    }
  t1=0;
  t2=0;
}
}

float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  return distance;
}

void lineFollow(){
  while (true){
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0){
    if (count == 1){ //turn right
    analogWrite(ENA, 70);
    analogWrite(ENB, 0);
    delay(1000);
    count = 0;
    }
    else { //stop
    analogWrite(ENB, 50);
    delay(100);
    stop();
    return;
    }
  }
  set_forward();
  PID_control();
  set_speed();
  }
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

  LMotorSpeed = baseSpeed + speedAdjust;
  RMotorSpeed = baseSpeed - speedAdjust;

  if (LMotorSpeed<10){
    LMotorSpeed = 10;
  }
  if (RMotorSpeed<10){
    RMotorSpeed = 10;
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
