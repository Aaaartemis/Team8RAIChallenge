#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>  // Include the Servo library
#include <QTRSensors.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int sensorPin[] = {A0, A2, A3, A4};  // Pins for 4 sensors
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

double time1 = 0;
double target = 12;

const int buttonPin = 4;     // the number of the pushbutton pin
const int ledPin =  LED_BUILTIN;      // the number of the LED pin

// variables will change:
int buttonState = 0;
int LEDstate = 1, pressed = 0;

int killed = 1;


const char* ssid     = "iPhone_200602152519";
const char* password = "20051107";


WiFiUDP udp;
const unsigned int localUdpPort = 55500;
typedef char PacketBuffer[255];
PacketBuffer incomingPacket;
const char* triggerCommand = "Stop";


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

  //while (!SerialUSB);          // wait for the PC to open the port
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
  myServo1.write(170);
  myServo2.write(10);
  myServo3.write(10);
  delay(1000);


  SerialUSB.println("Servos Initialized6");

  Serial.println("Button test start!");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

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
}

int turned = 2, tasking = 3;

void loop() {
  //buttonCheck();
  WifiCheck();
  killcheck();
  PID_control_wall();
  Serial.print("Distance to front: ");
  Serial.println(distance[1]);
  Serial.print("turned: ");
  Serial.println(turned);
  Serial.print("tasking: ");
  Serial.println(tasking);
  if (turned == 0 && tasking == 1) {
    Serial.println("Task 1(slope)");
    myServo.write(90);
    myServo1.write(110);
    myServo2.write(90);
    myServo3.write(100);
    time1 = millis();
    while (millis() <= time1 + 3000) {
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo1.write(170);
    myServo2.write(10);
    myServo3.write(10);
    tasking = 2;
  }
  if (turned == 1 && tasking == 2) {
    Serial.println("Task 2(stairs)");
    myServo1.write(180);
    myServo2.write(150);
    myServo3.write(60);
    delay(500);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    time1 = millis();
    while (millis() <= time1 + 7200) {
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo2.write(80);
    time1 = millis();
    while (millis() <= time1 + 3000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo3.write(110);
    while (millis() <= time1 + 2000) {
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo1.write(170);
    myServo2.write(10);
    myServo3.write(10);
    delay(1000);
    tasking = 3;
  }
  if (turned == 2 && tasking == 3) {
    Serial.println("Task 3(courseway)");
    myServo1.write(90);
    myServo2.write(180);
    myServo3.write(90);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    time1 = millis();
    while (millis() <= time1 + 3000) {
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    motoron1.setSpeed(1, -200);
    motoron1.setSpeed(2, -200);
    motoron2.setSpeed(1, -200);
    motoron2.setSpeed(2, -200);
    time1 = millis();
    while (millis() <= time1 + 200) {
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, -200);
      motoron1.setSpeed(2, -200);
      motoron2.setSpeed(1, -200);
      motoron2.setSpeed(2, -200);
    }
    myServo1.write(180);
    myServo2.write(130);
    myServo3.write(30);
    motoron1.setSpeed(1, 500);
    motoron1.setSpeed(2, 500);
    motoron2.setSpeed(1, 500);
    motoron2.setSpeed(2, 500);
    while (true) {
      readDistance(2);
      if (distance[2] <= 15)break;
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
    myServo1.write(170);
    myServo2.write(10);
    myServo3.write(10);
    delay(100);
    tasking = 4;
  }
  if (turned == 3 && tasking == 4) {
    Serial.println("Task 4(Lunar)");
    myServo1.write(90);
    myServo2.write(180);
    myServo3.write(90);
    delay(1000);
    time1 = millis();
    while (millis() <= time1 + 10000) {
      buttonCheck();
      killcheck();
      motoron1.setSpeed(1, 500);
      motoron1.setSpeed(2, 500);
      motoron2.setSpeed(1, 500);
      motoron2.setSpeed(2, 500);
    }
  }
  readDistance(1);
  if (distance[1] <= 3) {
    
    Serial.print("Distance to front: ");
    Serial.println(distance[1]);
    while (distance[1] <= 3) {
      motoron1.setSpeed(1, -400);
      motoron1.setSpeed(2, 400);
      Serial.println("Turning...");
      readDistance(1);
      Serial.print("Distance to front: ");
      Serial.println(distance[1]);
      readDistance(1);
    }
    turned += 1;
  }
}

void PID_control_wall() {
  readDistance(0);

  Serial.println();

  float error = distance[0] - target;
  Serial.print("Distance to left: ");
  Serial.println(distance[0]);

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);


  motoron1.setSpeed(1, motorspeeda);
  motoron1.setSpeed(2, motorspeedb);
}
