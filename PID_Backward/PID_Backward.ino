#include <QTRSensors.h>
#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>  // Include the Servo library
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* triggerCommand = "Print";

Servo myServo;      // Create a Servo object
Servo myServo1;
Servo myServo2;
Servo myServo3;

QTRSensors qtr;
const uint8_t SensorCount = 10;
uint16_t sensorValues[SensorCount];

// PID constants
float Kp = 0.05;
float Ki = 0;
float Kd = 0.6;
int P, I, D;
int lastError = 0;
int direction = 1;

// Motoron controllers with different I2C addresses
MotoronI2C motoron1(0x42);
MotoronI2C motoron2(0x10);

// Motor speeds
const int maxspeeda = 800;
const int maxspeedb = 800;
const int basespeeda = 277;
const int basespeedb = 277;

// Button pins
//int buttoncalibrate = 17; // A3
//int buttonstart = 2;

const char* ssid     = "iPhone_200602152519";
const char* password = "20051107";

WiFiUDP udp;
const unsigned int localUdpPort = 55500;
typedef char PacketBuffer[255];
PacketBuffer incomingPacket;

Adafruit_MPU6050 mpu;

double total = 0, normal, timeout;
unsigned long time1, time2;

double calibrateGyroZ() {
  double total = 0;
  Serial.println("Gyro Calibration Start");
  for (int i = 0; i < 300; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    total += g.gyro.z;
    delay(5);
    Serial.println(i);
  }
  Serial.println("Gyro Calibration End");
  return total / 300;
}

void calibration() {
  SerialUSB.println("Calibrating1");
  motoron1.setSpeed(1, 400);
  motoron1.setSpeed(2, -400);
  for (uint16_t i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(3);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  delay(2000);
  SerialUSB.println("Calibrating2");
  motoron1.setSpeed(1, -400);
  motoron1.setSpeed(2, 400);
  for (uint16_t i = 0; i < 80; i++) {
    qtr.calibrate();
    delay(3);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  delay(2000);
  SerialUSB.println("Calibrating3");
  motoron1.setSpeed(1, 400);
  motoron1.setSpeed(2, -400);
  for (uint16_t i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(3);
  }
  motoron1.setSpeed(1, 0);
  motoron1.setSpeed(2, 0);
  delay(2000);
  SerialUSB.println("Calibration Finished");
  incomingPacket[254] = '7';
}

void setup() {
  Wire.begin();
  SerialUSB.begin(115200);

  while (!SerialUSB);          // wait for the PC to open the port
  SerialUSB.println("Hello!");
  motoron1.setBus(&Wire);
  motoron2.setBus(&Wire);
  SerialUSB.println("Hello1!");
  qtr.setTypeRC();
  SerialUSB.println("Hello2");
  qtr.setSensorPins((const uint8_t[]) {
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31
  }, SensorCount);
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

  SerialUSB.println("Servos Initialized6");
  if (!mpu.begin()) {
    SerialUSB.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  SerialUSB.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  normal = calibrateGyroZ();
  SerialUSB.println("Gyro Calibrated");
  calibration();
}

void uTurnCheck() {
  bool flag = true;
  qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 800) {
      flag = false;
      timeout = 0;
      break;
    }
  }
  if (flag) {
    timeout += (millis() - time2);
    Serial.println(timeout);
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println("");
    if (timeout > 1000) {
      Serial.println("Line offed! Changing movement direction!");
      direction *= -1;
      timeout = 0;
    }
  }
}

void checkYFork() {

  int leftHits = 0, rightHits = 0, midHits=0;
  qtr.readLineBlack(sensorValues);
  // check sensors 0,1,2 for left; 7,8,9 for right
  for (int i = 0; i < 3; i++) {
    if (sensorValues[i] > 800) leftHits++;
    if (sensorValues[SensorCount - 1 - i] > 800) rightHits++;
  }
  for(int i=3;i<=6;i++){
    if(sensorValues[i]>800)midHits++;
  }
  if (leftHits >= 2 && rightHits >= 2 && midHits<=1) {
    if (direction == 1) {
      Serial.println("Y–fork detected! Taking left branch.");
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println("");
      motoron1.setSpeed(1, basespeeda);
      motoron1.setSpeed(2, 0);
      delay(300);
      motoron1.setSpeed(1, basespeeda);
      motoron1.setSpeed(2, basespeedb);
      delay(300);
    } else {
      Serial.println("Y–fork detected! Taking right branch.");
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println("");
      motoron1.setSpeed(1, 0);
      motoron1.setSpeed(2, basespeedb);
      delay(300);
      motoron1.setSpeed(1, basespeeda);
      motoron1.setSpeed(2, basespeedb);
      delay(300);
    }
    direction = 1;
    Serial.println("Going forward.");
    // reset PID
    lastError = 0;
    I = 0;
  }
}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (SensorCount - 1) * 500 - position;
  /*for (uint8_t i = 0; i < SensorCount; i++)
    {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    }
    SerialUSB.println("");
    Serial.print("Position: ");
    SerialUSB.println(position);
    Serial.print("  error: ");
    SerialUSB.println(error);*/

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;
  if (motorspeeda < 0)motorspeeda = 0;
  if (motorspeeda > maxspeeda)motorspeeda = maxspeeda;
  if (motorspeedb < 0)motorspeedb = 0;
  if (motorspeedb > maxspeedb)motorspeedb = maxspeedb;
  //motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  //motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  motorspeeda *= direction;
  motorspeedb *= direction;

  /*Serial.print("  Motor Change ");
    SerialUSB.println(motorspeed);
    Serial.println("  Motor speed ");
    SerialUSB.println(motorspeeda);
    SerialUSB.println(motorspeedb);*/
  motoron1.setSpeed(1, motorspeeda);
  motoron1.setSpeed(2, motorspeedb);
}

void loop() {
  time2 = millis();
  if (time2 % 1000 <= 5) {
    Serial.println("Print");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }Serial.println("");
  }
  
  PID_control();
  checkYFork();
  uTurnCheck();

  // UDP packet reading
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    Serial.print("Received packet: ");
    Serial.println(incomingPacket);

    if (strstr(incomingPacket, triggerCommand) != NULL) {
      qtr.readLineBlack(sensorValues);
      for (uint8_t i = 0; i < SensorCount; i++)
      {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
      }
      Serial.println("");
    }
  }
}
