/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo myservo2;

int pos = 0;    // variable to store the servo position
int pos2 = 0;

void setup() {
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(9); 
  myservo.write(0);
  myservo2.write(0);

  // myservo2.write(60);
  // delay(1000);
  // myservo2.write(0);

  // myservo.write(150);

}

void loop() {
    //myservo.write(100);
  // for (pos = 0; pos <= 150; pos += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
  // for (pos = 150; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15 ms for the servo to reach the position
  // }
}
