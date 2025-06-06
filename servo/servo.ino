#include <Servo.h>  // Include the Servo library

Servo myServo;      // Create a Servo object
Servo myServo1;      // Create a Servo object
Servo myServo2;      // Create a Servo object
Servo myServo3;      // Create a Servo object

void setup() {
  myServo.attach(50);
  myServo1.attach(51);
  myServo2.attach(52);
  myServo3.attach(53);

  myServo.write(90);
  myServo1.write(180);
  myServo2.write(0);
  myServo3.write(0);
}

void loop() {
  delay(100);
}
