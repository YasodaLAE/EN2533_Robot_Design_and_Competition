int val;
int preVal;
void setup() {
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  //Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  val=analogRead(A0);
  //Serial.println(val);
  delay(300);
  preVal=analogRead(A0);
  if (abs(val-preVal)>2){
    line_follow();
  }
  else{
    stop();
  }

}
