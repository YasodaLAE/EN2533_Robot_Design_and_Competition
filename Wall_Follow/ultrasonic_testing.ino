#define trigPin1     10                 // define pin connections for sensor and motor
#define echoPin1     3

#define trigPin2    11
#define echoPin2    12

#define in1      9      
#define in2      13     
#define in3      7 
#define in4      8  
#define en1      6
#define en2      5

long duration;


void setup() {
  // put your setup code here, to run once:
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(en1, 90);
  analogWrite(en2, 90);
}

int distance1, distance2;

void loop() {
  // put your main code here, to run repeatedly:
  distance1 = distance(trigPin1, echoPin1);
  distance2 = distance(trigPin2, echoPin2);
  Serial.print("distance1 ");
  Serial.println(distance1);
  Serial.print("distance2 ");
  Serial.println(distance2);
  //delay(50);
  // if (distance1>= 10 && distance2>=10 && distance2 <= 25){
  //   goForward();
  // }
  if (distance1 < 20){
    stop();
    delay(3000);
    turnRight();
    delay(500);
    //goForward();

  }
  
  if (distance2 < 10){
    turnRight();
    delay(40);
    //goForward();

  }
  distance1 = distance(trigPin1, echoPin1);
  distance2 = distance(trigPin2, echoPin2);
  
  if (distance2 > 25){
    turnLeft();
    delay(40);
    //goForward();
  }
  goForward();
}

void moveBack(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnRight(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turnLeft(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void goForward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(en1, 150);
  analogWrite(en2, 75);
}

int distance(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int dur = pulseIn(echoPin,HIGH); 
  int distance =  dur * 0.034/2 ;
  delay(200);
  return distance;
}