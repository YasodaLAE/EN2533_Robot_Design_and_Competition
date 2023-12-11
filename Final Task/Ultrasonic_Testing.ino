int duration = 0;
float distance = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A2, OUTPUT);
  pinMode(A3, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readUltrasonic(A2,A3);
}
float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  // delay(1000);
  Serial.println(distance);
  return distance;
}