#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUDP.h>

MotoronI2C motoron1(0x10);
MotoronI2C motoron2(0x42);
const int buttonPin = 4;
const int ledPin =  LED_BUILTIN; 

const char* ssid     = "PhaseSpaceNetwork_2.4G";
const char* password = "8igMacNet";

// Static IP configuration
//IPAddress local_IP(172, 20, 10, 14);
//IPAddress gateway(172, 20, 10, 1);
//IPAddress subnet(255, 255, 255, 240);
//IPAddress primaryDNS(172, 20, 10, 1);

WiFiUDP udp;
const unsigned int localUdpPort = 55500;
typedef char PacketBuffer[255];
PacketBuffer incomingPacket;
const char* triggerCommand = "Stop";

Servo myServo;      // Create a Servo object
Servo myServo1;
Servo myServo2;
Servo myServo3;
int pos = 0; 

int buttonState = 0; 
int state=1,pressed=0;

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

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  myServo.attach(23);
  myServo1.attach(25);
  myServo2.attach(27);
  myServo3.attach(29);
  
  Serial.println("Initialize end!");

  //Serial.println("Configuring static IP...");
  //WiFi.config(local_IP, primaryDNS, gateway, subnet);

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

  myServo.write(0);
  myServo1.write(0);
  myServo2.write(190);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  if (state==1){
    // turn LED off:
    digitalWrite(ledPin, LOW);
    if(millis()%4000<=1000){
      //Forwards
      Serial.println("Forward");
      motoron1.setSpeed(1, 800);
      motoron1.setSpeed(2, 800); 
      motoron2.setSpeed(1, 800); 
      motoron2.setSpeed(2, 800); 
    }else if(millis()%4000<=2000){
      //Backwards
      Serial.println("Backward");
      motoron1.setSpeed(1, -800);
      motoron1.setSpeed(2, -800);
      motoron2.setSpeed(1, -800);
      motoron2.setSpeed(2, -800);
    }else if(millis()%4000<=3000){
      //Left turn
      Serial.println("Left");
      motoron1.setSpeed(1, -800);
      motoron1.setSpeed(2, 800);
      motoron2.setSpeed(1, -800);
      motoron2.setSpeed(2, 800);
    }else{
      //Right turn
      Serial.println("Right");
      motoron1.setSpeed(1, 800);
      motoron1.setSpeed(2, -800);
      motoron2.setSpeed(1, 800);
      motoron2.setSpeed(2, -800);
    }
  }else{
    // turn LED on:
    Serial.println(state);
    digitalWrite(ledPin, HIGH);
    motoron1.setSpeed(1, 0);
    motoron1.setSpeed(2, 0);
    motoron2.setSpeed(1, 0);
    motoron2.setSpeed(2, 0);
  }
  if (buttonState == LOW&&pressed==0) {
    state*=-1;
    pressed=1;
  } else if(buttonState==HIGH){
    pressed=0;
  }
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
      state=-1;
    }
  }
  delay(10);
}
