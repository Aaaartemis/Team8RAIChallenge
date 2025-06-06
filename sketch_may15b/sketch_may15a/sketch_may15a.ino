const int sensorPin[ ] = {A0,A1,A2,A3};
float distance[4];
const int AVERAGE_OF =50;
const float MCU_VOLTAGE = 3.3;

void setup()
{
  Serial.begin(9600); 
  Serial.println("Robojax Sharp GP2Y0A51SK0F demo");
}//setup ends here


void loop(){
   readDistance(0);//read sensor 1

  // print out the value you read:
  Serial.print("d1 =");// 
  Serial.print(distance[0]);
  Serial.print("   ");
     readDistance(1);//read sensor 1

  // print out the value you read:
  Serial.print("d2 =");// 
  Serial.print(distance[1]);
  Serial.print("   ");
     readDistance(2);//read sensor 1

  // print out the value you read:
  Serial.print("d3 =");// 
  Serial.print(distance[2]);
  Serial.print("   ");
     readDistance(3);//read sensor 1

  // print out the value you read:
  Serial.print("d4 =");// 
  Serial.print(distance[3]);
  Serial.println("   ");
  delay(300);
}
void readDistance(int sensor)
{
  //Robojax.com code for sharp IR sensor 
      float voltage_temp_average=0;
      
      for(int i=0; i < AVERAGE_OF; i++)
    {
      int sensorValue = analogRead(sensorPin[sensor] );
      delay(1);      
      voltage_temp_average +=sensorValue * (MCU_VOLTAGE / 1023.0);

    }
     voltage_temp_average /= AVERAGE_OF;

  // eqution of the fitting curve
  ////33.9 + -69.5x + 62.3x^2 + -25.4x^3 + 3.83x^4
  distance[sensor] = 33.9 + -69.5*(voltage_temp_average) + 62.3*pow(voltage_temp_average,2) + -25.4*pow(voltage_temp_average,3) + 3.83*pow(voltage_temp_average,4);
  //distance[sensor]=voltage_temp_average;
}//readDistance
