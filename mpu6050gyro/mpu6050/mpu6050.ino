  // Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

double total=0,normal;
unsigned long time1;


double calibrateZ(){
  double total=0;
  Serial.println("Calibration Start");
  for(int i=0;i<300;i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    total+=g.gyro.z;
    delay(5);
  }
  Serial.println("Calibration End");
  return total/300;
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  normal=calibrateZ();

  Serial.println(normal);
  delay(100);
}

void loop() {
  time1=micros();
  //delay(1);
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */

  Serial.print("Rotation Z: ");
  Serial.print(g.gyro.z);
  Serial.print(" rad/s   ");
  Serial.print(g.gyro.z/3.1416*180);
  Serial.println(" deg/s");

  Serial.print("Change: ");
  Serial.print(g.gyro.z-normal);
  Serial.print(" rad/s   ");
  Serial.print((g.gyro.z-normal)/3.1416*180);
  Serial.println(" deg/s");

  Serial.print("Angle: ");
  Serial.print(total);
  Serial.print(" rads;  ");
  Serial.print(total/3.1416*180);
  Serial.print(" degs;  ");
  
  Serial.println("");

  total+=(g.gyro.z-normal)*(micros()-time1)/1000000;
}
