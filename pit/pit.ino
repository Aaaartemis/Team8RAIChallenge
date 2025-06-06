#include <Servo.h>  // Include the Servo library
#include <Wire.h>
#include <Motoron.h>


MotoronI2C motoron1(0x42);
MotoronI2C motoron2(0x10);

Servo myServo;      // Create a Servo object
Servo myServo1;
Servo myServo2;
Servo myServo3;
int pos = 0;        // Variable to store the servo position

void setup() {

  Wire.begin();
  SerialUSB.begin(9600);

  //while(!SerialUSB);           // wait for the PC to open the port
  SerialUSB.println("Hello!");
  motoron1.setBus(&Wire);
  motoron2.setBus(&Wire);

  SerialUSB.println("Motoron Initialized1");
  // Reinitialize both Motorons
  motoron1.reinitialize();
  motoron2.reinitialize();

  SerialUSB.println("Motoron Initialized2");
  motoron1.disableCrc();
  motoron2.disableCrc();
  motoron1.clearResetFlag();
  motoron2.clearResetFlag();


  SerialUSB.println("Motoron Initialized3");
  motoron1.setMaxAcceleration(1, 140);
  motoron1.setMaxDeceleration(1, 300);
  motoron1.setMaxAcceleration(2, 140);
  motoron1.setMaxDeceleration(2, 300);
  motoron2.setMaxAcceleration(1, 140);
  motoron2.setMaxDeceleration(1, 300);
  motoron2.setMaxAcceleration(2, 140);
  motoron2.setMaxDeceleration(2, 300);
  SerialUSB.println("Motoron Initialized4");
  // Set all motors to speed 0 initially
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  motoron2.setSpeed(1, 0);
  motoron2.setSpeed(2, 0);

  SerialUSB.println("Motoron Initialized5");


  myServo.attach(50);
  myServo1.attach(51); // Attach the servo to pin 9 (you can change this)
  myServo2.attach(52);
  myServo3.attach(53);
  myServo.write(90);
  myServo1.write(110);
  myServo2.write(90);
  myServo3.write(100);
  delay(15);
}

void loop() {
  motoron1.setSpeed(1, 500);
  motoron1.setSpeed(2, 500);
  motoron2.setSpeed(1, 500);
  motoron2.setSpeed(2, 500);
  if(millis()>=5000)myServo3.write(180);
  if(millis()>=7000)myServo3.write(100);
  //delay(9999);
  delay(100);
}
