const int sensorPin[] = {A0, A1, A2, A3};  // Pins for 4 sensors
float distance[4];                         // Store distances for each sensor
const int AVERAGE_OF = 50;
const float MCU_VOLTAGE = 3.3;

void setup() {
  Serial.begin(9600);
  Serial.println("GP2Y0A51SK0F - 4 Sensor Setup");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    readDistance(i);  // Read each sensor
  }

  // Print results
  for (int i = 0; i < 4; i++) {
    Serial.print("d");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.print(distance[i]);
    Serial.print(" cm\t");
  }
  Serial.println();
  delay(300);
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
