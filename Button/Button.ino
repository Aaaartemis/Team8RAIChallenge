#include <Wire.h>

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 4;     // the number of the pushbutton pin
const int ledPin =  LED_BUILTIN;      // the number of the LED pin

// variables will change:
int buttonState = 0; 
int LEDstate=1,pressed=0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Button test start!");
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
  if (LEDstate==1){
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  }else{
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
  if (buttonState == LOW&&pressed==0) {
    LEDstate*=-1;
    pressed=1;
    
  } else if(buttonState==HIGH){
    pressed=0;
  }
}
