const uint8_t sensorPins[] = {22,23,24,25,26,27,28,29,30,31,32,33,34}; // GPIOs connected to the 9 sensor outputs
const uint8_t SensorCount = 13;
uint16_t sensorValues[SensorCount];

void readQTR_RC(uint16_t* readings) {
  // 1. Charge capacitors
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }
  delayMicroseconds(10);

  // 2. Set to input (start discharge)
  for (uint8_t i = 0; i < SensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // 3. Measure time to LOW
  unsigned long startTime = micros();
  unsigned long timeout = 3000; // Max wait time in µs
  bool done[SensorCount] = {false};

  while ((micros() - startTime) < timeout) {
    for (uint8_t i = 0; i < SensorCount; i++) {
      if (!done[i] && digitalRead(sensorPins[i]) == LOW) {
        readings[i] = micros() - startTime;
        done[i] = true;
      }
    }
  }

  // 4. Default to timeout if not discharged
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (!done[i]) {
      readings[i] = timeout;
    }
  }
}

void setup() {
  Serial.begin(115200); // Start serial monitor
  delay(1000); // Give time for monitor to connect
  Serial.println("QTR-HD-09RC Test Start");
  pinMode(26, OUTPUT);
  delay(10);
  digitalWrite(26, HIGH);
}

void loop() {
  readQTR_RC(sensorValues); // Read all sensors

  // Print values
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  delay(99); // Slow down printing
}
