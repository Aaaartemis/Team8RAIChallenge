#include <WiFi.h>
#include <WiFiUDP.h>

// Wi-Fi credentials
const char* ssid     = "iPhone_200602152519";
const char* password = "20051107";

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

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(1); }  
  pinMode(LED_BUILTIN, OUTPUT);

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

  // 打印网络信息
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());

  // 启动 UDP 监听
  udp.begin(localUdpPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localUdpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0';
    }
    Serial.print("Received packet: ");
    Serial.println(incomingPacket);

    if (strstr(incomingPacket, triggerCommand) != NULL) {
      Serial.println("Trigger command detected!");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
  delay(10);
}
