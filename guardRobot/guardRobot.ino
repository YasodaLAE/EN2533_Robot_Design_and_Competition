#define IR0 22
#define IR1 A0//11
#define IR2 A1//12
#define IR3 A2//13
#define IR4 A3//2
#define IR5 A4//3
#define IR6 A5//4
#define IR7 23

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

int IR_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {0, -15, -10, -5, 5, 10, 15, 0};

int directionArray[5] = {-1,-1,-1,-1};
int right[5] = {1,0,0,1};
int left[5] = {0,1,1,0};

int RMotor = 0;
int LMotor = 0;
int speedAdjust = 0;
int baseSpeed = 85; //final = 75

int count = 1;
int i = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 12;//{5.5-,1+,3++,4+} final = 3.5
float Kd = 0.1;//10, 1 final = 6
float Ki = 0; //final = 0
unsigned long start_time; 
float max_distance = 10;
int t1;
int t2;

void setup() {
 // Serial.begin(9600);
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

  pinMode(trig1, OUTPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(echo2, INPUT);
  start_time = millis();
}
//int count2 = 1;
void loop() {
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
      analogWrite(ENA, 60);
      analogWrite(ENB, 60);
      set_forward();           // if right sensor triggered first
      if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
        turnRight();
        delay(1000);
      }
      analogWrite(ENA, 80);
      analogWrite(ENB, 80);
      set_forward();
      //lineFollow();
          // direction is right to left
    }
  else{
    //Serial.println(" ");
  }
  t1=0;
  t2=0;
  
}
}

void turnRight(){
  analogWrite(ENA, 50);
    analogWrite(ENB, 0);

    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
  // delay(1500);
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

void lineFollow(){
  while (true){
  read_IR();
  if (IR_val[0]==0 && IR_val[1]==0 && IR_val[2]==0 && IR_val[3]==0 && IR_val[4]==0 && IR_val[5]==0 && IR_val[6]==0 && IR_val[7]==0){
    if (count == 1){ //turn right
    analogWrite(ENA, 70);
    analogWrite(ENB, 0);
    delay(1000);
    count = 0;
    }
    else { //stop
    // analogWrite(ENB, 50);
    // delay(100);
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

  Serial.print(IR_val[0]);
  Serial.print(IR_val[1]);
  Serial.print(IR_val[2]);
  Serial.print(IR_val[3]);
  Serial.print(IR_val[4]);
  Serial.print(IR_val[5]);
  Serial.print(IR_val[6]);
  Serial.println(IR_val[7]);
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