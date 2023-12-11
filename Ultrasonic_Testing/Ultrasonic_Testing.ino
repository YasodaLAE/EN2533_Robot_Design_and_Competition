long unsigned duration = 0;
float distance = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(48 ,OUTPUT);
  pinMode(49, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readUltrasonic(48,49);
}
float readUltrasonic(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  // delay(1000);
  Serial.println(distance);
  return distance;
}