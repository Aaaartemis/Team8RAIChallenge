#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron1(0x42);
MotoronI2C motoron2(0x10);

void setup() {
  Wire.begin();
  SerialUSB.begin(9600);

  while(!SerialUSB);           // wait for the PC to open the port
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
}

void loop() {
  // Run motors
  motoron1.setSpeed(1, 900);
  motoron1.setSpeed(2, 900);
  motoron2.setSpeed(1, 900);
  motoron2.setSpeed(2, 900);     
  delay(500);
  SerialUSB.println("Motoron Initialized6");
  /*motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  motoron2.setSpeed(1, 0);
  motoron2.setSpeed(2, 0);
  delay(500);*/
}
