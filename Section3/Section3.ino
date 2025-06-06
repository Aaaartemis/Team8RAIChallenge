#include <QTRSensors.h>
#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>  // Include the Servo library
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Servo myServo;      // Create a Servo object
Servo myServo1;
Servo myServo2;
Servo myServo3;

QTRSensors qtr;
QTRSensors qtrBack;
const uint8_t SensorCount = 10;
const uint8_t BackSensorCount = 3;
uint16_t sensorValues[SensorCount];
uint16_t BacksensorValues[BackSensorCount];

// PID constants
float Kp = 0.08;
float Ki = 0;
float Kd = 0.6;
int P, I, D;
int lastError = 0;

MotoronI2C motoron1(0x42);
MotoronI2C motoron2(0x10);

const int sensorPin[ ] = {A0, A1, A2, A3};
float distance[4];
const int AVERAGE_OF = 50;
const float MCU_VOLTAGE = 3.3;

// Motor speeds
const int maxspeeda = 800;
const int maxspeedb = 800;
const int basespeeda = 240;
const int basespeedb = 240;
const float speednow = 0.7;

const int buttonPin = 4;     // the number of the pushbutton pin
const int ledPin =  LED_BUILTIN;      // the number of the LED pin

// variables will change:
int buttonState = 0;
int LEDstate = 1, pressed = 0;

int killed = 1;

bool ForkDetected = false;

const char* ssid     = "iPhone_200602152519";
const char* password = "20051107";

WiFiUDP udp;
const unsigned int localUdpPort = 55500;
typedef char PacketBuffer[255];
PacketBuffer incomingPacket;
const char* triggerCommand = "Stop";

Adafruit_MPU6050 mpu;

double total = 0, normal, timeout;
unsigned long time1, time2;
void readDistance(int sensor)
{
  //Robojax.com code for sharp IR sensor
  float voltage_temp_average = 0;

  for (int i = 0; i < AVERAGE_OF; i++)
  {
    int sensorValue = analogRead(sensorPin[sensor] );
    delay(1);
    voltage_temp_average += sensorValue * (MCU_VOLTAGE / 1023.0);

  }
  voltage_temp_average /= AVERAGE_OF;

  // eqution of the fitting curve
  ////33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
  distance[sensor] = 33.9 + -69.5 * (voltage_temp_average) + 62.3 * pow(voltage_temp_average, 2) + -25.4 * pow(voltage_temp_average, 3) + 3.83 * pow(voltage_temp_average, 4);
  //distance[sensor]=voltage_temp_average;
}//readDistance
void killcheck() {
  if (killed == 1) {
    while (killed != -1) {
      motoron1.setSpeed(1, 0);
      motoron1.setSpeed(2, 0);
      SerialUSB.println("Dead!");
      buttonCheck();
    }
  }
}
void WifiCheck() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    Serial.print("Received packet: ");
    Serial.println(incomingPacket);

    if (strstr(incomingPacket, triggerCommand) != NULL) {
      Serial.println("NOOOOOO!!!!");
      killed = 1;
    }
  }
}
void buttonCheck() {
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  if (LEDstate == 1) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
  if (buttonState == LOW && pressed == 0) {
    LEDstate *= -1;
    killed *= -1;
    pressed = 1;

  } else if (buttonState == HIGH) {
    pressed = 0;
  }
}
void calibration() {
  SerialUSB.println("Calibrating1");
  for (uint16_t i = 0; i < 15; i++) {
    motoron1.setSpeed(1, 200);
    motoron1.setSpeed(2, -200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  SerialUSB.println("Calibrating2");
  for (uint16_t i = 0; i < 30; i++) {
    motoron1.setSpeed(1, -200);
    motoron1.setSpeed(2, 200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  SerialUSB.println("Calibrating3");
  for (uint16_t i = 0; i < 30; i++) {
    motoron1.setSpeed(1, 200);
    motoron1.setSpeed(2, -200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  SerialUSB.println("Calibrating2");
  for (uint16_t i = 0; i < 30; i++) {
    motoron1.setSpeed(1, -200);
    motoron1.setSpeed(2, 200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  SerialUSB.println("Calibrating3");
  for (uint16_t i = 0; i < 30; i++) {
    motoron1.setSpeed(1, 200);
    motoron1.setSpeed(2, -200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  for (uint16_t i = 0; i < 15; i++) {
    motoron1.setSpeed(1, -200);
    motoron1.setSpeed(2, 200);
    qtr.calibrate();
    delay(2);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  SerialUSB.println("Calibration Finished");
  SerialUSB.println("Sensor Calibration Results:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    SerialUSB.print("Sensor ");
    SerialUSB.print(i);
    SerialUSB.print(": min=");
    SerialUSB.print(qtr.calibrationOn.minimum[i]);
    SerialUSB.print(" max=");
    SerialUSB.println(qtr.calibrationOn.maximum[i]);
  }

  delay(2000);
}

int stopCount = 0;
void stopcheck() {
  qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 700) {
      return;
    }
  }
  /*stopCount += 1;
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    delay(200);
    if (stopCount == 1) {
    myServo.write(90);
    myServo1.write(110);
    myServo2.write(90);
    myServo3.write(100);
    delay(15);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    while (true) {
      qtr.readLineBlack(sensorValues);
      for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 700) {
          return;
        }
      }
    }
    }
    if (stopCount == 2) {
    myServo.write(90);
    myServo1.write(110);
    myServo2.write(90);
    myServo3.write(100);
    delay(15);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    double time1 = millis();
    while (millis() <= time1 + 5000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo3.write(180);
    time1 = millis();
    while (millis() <= time1 + 2000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo3.write(100);
    while (true) {
      qtr.readLineBlack(sensorValues);
      for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 700) {
          return;
        }
      }
    }
    }
    if (stopCount == 3) {
    myServo.write(0);
    delay(3000);
    for (int i = 90; i >= 0; i--) {
      myServo.write(i);
      delay(10);
    }
    delay(2000);
    while (true) {
      motoron1.setSpeed(1, 900);
      motoron1.setSpeed(2, 900);
      delay(500);
      myServo2.write(0);
      myServo3.write(0);
      delay(500);
      myServo2.write(90);
      myServo3.write(90);
    }
    }*/
  myServo.write(0);
  delay(1000);
  for (int i = 90; i >= 0; i--) {
    myServo.write(i);
    delay(10);
  }
  delay(2000);
  while (true) {
    motoron1.setSpeed(1, 900);
    motoron1.setSpeed(2, 900);
    delay(500);
    myServo2.write(0);
    myServo3.write(0);
    delay(500);
    myServo2.write(90);
    myServo3.write(90);
  }
}
//double time2;
void setup() {
  Wire.begin();
  SerialUSB.begin(115200);
  //while (!SerialUSB);
  SerialUSB.println("Hello!");
  motoron1.setBus(&Wire);
  motoron2.setBus(&Wire);
  SerialUSB.println("Hello1!");
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31
  }, SensorCount);
  SerialUSB.println("Hello2");
  qtrBack.setTypeRC();
  qtrBack.setSensorPins((const uint8_t[]) {
    32, 33, 34
  }, BackSensorCount);
  SerialUSB.println("Hello3");
  motoron1.reinitialize();
  SerialUSB.println("Motoron Initialized2");
  motoron1.disableCrc();
  motoron1.clearResetFlag();
  motoron2.disableCrc();
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
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  motoron2.setSpeed(1, 0);
  motoron2.setSpeed(2, 0);

  SerialUSB.println("Motoron Initialized5");

  myServo.attach(50);
  myServo1.attach(51);
  myServo2.attach(52);
  myServo3.attach(53);
  myServo.write(90);
  myServo1.write(90);
  myServo2.write(90);
  myServo3.write(90);
  delay(1000);

  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi!");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());

  udp.begin(localUdpPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localUdpPort);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  Serial.println("Button test start!");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);


  motoron1.setSpeed(1, 500);
  motoron1.setSpeed(2, 500);
  motoron2.setSpeed(1, 500);
  motoron2.setSpeed(2, 500);
  time2 = millis();
  while (millis() <= time2 + 3000) {
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
  }
  motoron1.setSpeed(1, 000);
  motoron1.setSpeed(2, 000);
  motoron2.setSpeed(1, 000);
  motoron2.setSpeed(2, 000);
  myServo.write(90);
  myServo1.write(180);
  myServo2.write(0);
  myServo3.write(10);
  delay(500);
  calibration();
}


void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = ((SensorCount - 1) * 500) - position;

  // -- PID --
  float Pterm = error;
  static float Iterm = 0;
  static int lastError = 0;
  Iterm += error;
  float Dterm = error - lastError;
  lastError = error;

  float correction = Kp * Pterm + Ki * Iterm + Kd * Dterm;

  int speedA = basespeeda - (int)correction;
  int speedB = basespeedb + (int)correction;

  speedA = constrain(speedA, 0, maxspeeda);
  speedB = constrain(speedB, 0, maxspeedb);

  speedA *= speednow;
  speedB *= speednow;

  motoron1.setSpeed(1, speedA);
  motoron1.setSpeed(2, speedB);

  SerialUSB.print("Pos="); SerialUSB.print(position);
  SerialUSB.print("  Err="); SerialUSB.print(error);
  SerialUSB.print("  Corr="); SerialUSB.print(correction);
  SerialUSB.print("  A="); SerialUSB.print(speedA);
  SerialUSB.print("  B="); SerialUSB.println(speedB);

  SerialUSB.print("Sensor Values: ");
  for (uint8_t i = 0; i < SensorCount; i++) {
    SerialUSB.print(sensorValues[i]);
    SerialUSB.print('\t');
  }
  /*SerialUSB.print('\t');
    SerialUSB.print('\t');
    for (uint8_t i = 0; i < BackSensorCount; i++) {
    SerialUSB.print(BacksensorValues[i]);
    SerialUSB.print('\t');
    }*/
  SerialUSB.println();
}

int tasked = 0;

void loop() {
  readDistance(2);
  if (distance[2] >= 10)tasked = 1;
  //WifiCheck();
  //buttonCheck();
  //killcheck();
  if (tasked == 1) {
    myServo.write(90);
    myServo1.write(110);
    myServo2.write(90);
    myServo3.write(100);
    delay(15);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    double time1 = millis();
    while (millis() <= time1 + 5000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo3.write(180);
    time1 = millis();
    while (millis() <= time1 + 2000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo3.write(100);
    delay(1000);
    bool flag = true;
    while (flag) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
      qtr.readLineBlack(sensorValues);
      for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 700) {
          tasked = 2;
          flag = false;
        }
      }
    }
    myServo.write(90);
    myServo1.write(180);
    myServo2.write(0);
    myServo3.write(10);
    delay(500);
    calibration();
  }
  PID_control();     // normal line following
  stopcheck();
}
