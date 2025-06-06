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

  for (uint16_t i = 0; i < 20; i++) {
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
  stopCount += 1;
  motoron1.setSpeed(1, 500);
  motoron1.setSpeed(2, 500);
  delay(200);
  if (stopCount == 1) {
    calibration();
    return;
  }
}


void setup() {
  Wire.begin();
  SerialUSB.begin(115200);
  //while (!SerialUSB);
  SerialUSB.println("Hello!");
  motoron1.setBus(&Wire);
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


  SerialUSB.println("Motoron Initialized3");
  motoron1.setMaxAcceleration(1, 300);
  motoron1.setMaxDeceleration(1, 300);
  motoron1.setMaxAcceleration(2, 300);
  motoron1.setMaxDeceleration(2, 300);
  SerialUSB.println("Motoron Initialized4");
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);

  SerialUSB.println("Motoron Initialized5");

  myServo.attach(50);
  myServo1.attach(51);
  myServo2.attach(52);
  myServo3.attach(53);
  myServo.write(90);
  myServo1.write(180);
  myServo2.write(0);
  myServo3.write(10);
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
  calibration();

  Serial.println("Button test start!");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void uTurn() {
  motoron1.setSpeed(1, 400);
  motoron1.setSpeed(2, 400);
  ForkDetected = false;
  bool flag = true;
  Serial.println("No Line found for 1000ms! Turning Back!");
  total = 0;
  while (flag) {
    WifiCheck();
    buttonCheck();
    killcheck();
    qtr.readLineBlack(sensorValues);
    motoron1.setSpeed(1, 200);
    motoron1.setSpeed(2, -200);
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (sensorValues[i] > 700) {
        flag = false;
        break;
      }
    }
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println("");
  }

  Serial.println("U-Turn Finished!");

  lastError = 0;
  I = 0;
  delay(500);
}

void uTurnCheck() {
  bool flag = true;
  qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 700) {
      flag = false;
      timeout = 0;
      break;
    }
  }
  if (flag) {
    timeout += (millis() - time2);
    Serial.println(timeout);
    Serial.println("");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println("");
    if (timeout > 1000) {
      timeout = 0;
      uTurn();
    }
  }
}

void checkYFork() {
  int leftHits = 0, rightHits = 0, midHits = 0, backHits = 0;
  qtr.readLineBlack(sensorValues);
  qtrBack.readLineBlack(BacksensorValues);
  for (int i = 0; i < 3; i++) {
    if (sensorValues[i] > 700) leftHits++;
    if (sensorValues[SensorCount - 1 - i] > 700) rightHits++;
  }
  for (int i = 3; i <= 6; i++) {
    if (sensorValues[i] > 700)midHits++;
  }
  for (int i = 0; i < 3; i++) {
    if (BacksensorValues[i] > 700) backHits++;
  }
  if (leftHits >= 1 && rightHits >= 1 && midHits <= 1) {
    Serial.println("Y–fork detected! Taking left branch.");
    ForkDetected = true;
    myServo.write(0);
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println("");
    for (uint8_t i = 0; i < BackSensorCount; i++)
    {
      Serial.print(BacksensorValues[i]);
      Serial.print('\t');
    }
    Serial.println("");
    motoron1.setSpeed(1, basespeeda + 100);
    motoron1.setSpeed(2, 0);
    delay(200);
    motoron1.setSpeed(1, basespeeda);
    motoron1.setSpeed(2, basespeedb);
    delay(200);
    myServo.write(90);
    // reset PID
    lastError = 0;
    I = 0;
  }
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

void loop() {
  time2 = millis();

  WifiCheck();
  buttonCheck();
  killcheck();
  stopcheck();

  PID_control();     // normal line following
  checkYFork();      // detect and take fork
  //if (ForkDetected)uTurnCheck();     // detect and recover from line loss
}
