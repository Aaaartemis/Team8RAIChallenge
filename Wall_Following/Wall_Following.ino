#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>  // Include the Servo library

const int sensorPin[] = {A0, A1, A2, A3};  // Pins for 4 sensors
float distance[4];                         // Store distances for each sensor
const int AVERAGE_OF = 50;
const float MCU_VOLTAGE = 3.3;

Servo myServo;      // Create a Servo object
Servo myServo1;
Servo myServo2;
Servo myServo3;

const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

// PID constants
float Kp = 14;
float Ki = 0;
float Kd = 6;
int P, I, D;
int lastError = 0;
bool onoff = false;

double target=12;

// Motoron controllers with different I2C addresses
MotoronI2C motoron1(0x42);
MotoronI2C motoron2(0x10);

// Motor speeds
const int maxspeeda = 800;
const int maxspeedb = 800;
const int basespeeda = 400;
const int basespeedb = 400;

// Button pins
//int buttoncalibrate = 17; // A3
//int buttonstart = 2;

void readDistance(int sensor) {
  float voltage_temp_average = 0;

  for (int i = 0; i < AVERAGE_OF; i++) {
    int sensorValue = analogRead(sensorPin[sensor]);
    delay(1);
    voltage_temp_average += sensorValue * (MCU_VOLTAGE / 1023.0);
  }
  voltage_temp_average /= AVERAGE_OF;

  // Polynomial fitting curve equation for GP2Y0A51SK0F
  distance[sensor] = 33.9 
                   - 69.5 * voltage_temp_average 
                   + 62.3 * pow(voltage_temp_average, 2) 
                   - 25.4 * pow(voltage_temp_average, 3) 
                   + 3.83 * pow(voltage_temp_average, 4);
}

void setup() {
  Wire.begin();
  SerialUSB.begin(115200);

  while (!SerialUSB);          // wait for the PC to open the port
  SerialUSB.println("Hello2!");
  motoron1.setBus(&Wire);
  motoron2.setBus(&Wire);
  SerialUSB.println("Hello3");

  // Reinitialize both Motorons
  motoron1.reinitialize();
  motoron2.reinitialize();

  SerialUSB.println("Motoron Initialized2");
  motoron1.disableCrc();
  motoron2.disableCrc();
  motoron1.clearResetFlag();
  motoron2.clearResetFlag();


  SerialUSB.println("Motoron Initialized3");
  motoron1.setMaxAcceleration(1, 300);
  motoron1.setMaxDeceleration(1, 300);
  motoron1.setMaxAcceleration(2, 300);
  motoron1.setMaxDeceleration(2, 300);
  motoron2.setMaxAcceleration(1, 300);
  motoron2.setMaxDeceleration(1, 300);
  motoron2.setMaxAcceleration(2, 300);
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
  myServo1.write(180);
  myServo2.write(0);
  myServo3.write(0);
  delay(1000);


  SerialUSB.println("Servos Initialized6");
}

void loop() {
  PID_control_wall();
}

void PID_control_wall() {
  readDistance(0);  

  Serial.println();
  
  float error=distance[0]-target;
  Serial.print("Distance to left: ");
  Serial.println(distance[0]);
  //Serial.print("Error: ");
  //Serial.println(error);

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);
  //Serial.print("  Motor Change ");
  //SerialUSB.println(motorspeed);

  //Serial.println("  Motor speed ");
  //SerialUSB.println(motorspeeda);
  //SerialUSB.println(motorspeedb);

  readDistance(1);
  Serial.print("Distance to front: ");
  Serial.println(distance[1]);
  if(distance[1]<=15){
    motorspeedb+=375;
    motorspeeda-=375;
  }

  motoron1.setSpeed(1, motorspeeda);
  motoron1.setSpeed(2, motorspeedb);
}
