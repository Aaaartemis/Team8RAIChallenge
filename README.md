# **Read Me**

## Transformer Robot Project

This is the Transformer Robot project - the highest scoring team (out of 11 competitors) for the final UCL year 1 Robotics & Artificial Intelligence challenge! Transformer is designed to adapt to different challenges by changing its shape. Built using Arduino-compatible components and programmed in Arduino IDE, this robot utilizes servos, DC motors, PID control, and multiple sensors to accomplish a variety of missions such as line and wall following (heavily programming and control based) and other mostly mechanically tailored challenges. 

## Features

Line Following using infrared reflectance sensor arrays (Pololu QTR)

Wall Following with distance sensors

PID Control for both motor speed and directional accuracy (for line and wall following)

Gyroscope for orientation using MPU6050

Servo-Controlled Mechanisms for transformation between different configurations

Advanced Mobility: Includes handling tasks like:

  -Stairs climbing

  -Zipline
  
  -Lava pit 
  
  -Lunar surface 
  
  -Treadmill 

## Folder Overview

Folder/File             Description

Motors 		             - DC motor control code, basic movement logic

Servo 			           - Servo motor control for transformation or articulation

PIDnew		             - Improved PID with Y-fork and U-turn

PIDwifiGyro		         - PID control using gyroscope and optional Wi-Fi features

Wall_Following	       - Wall following implementation using distance sensors

PID_Backward	         - PID-based reverse driving logic

Mpu6050gyro 	         - Gyro testing and calibration

Pit 			             - Lava pit navigation code

Stairs			           - Stair navigation code

Zipline 		           - Zipline navigation code

Causeway		           - Course transition or guided path logic

Section1/2/3/		       - Code for each section 

Button 		             - Code for the button

sketch_*.ino/		       - Arduino sketches for various experiments or finalized logic

Pololu QTR-8...fzpz	   - Fritzing sketches (unrelated to code)


## Getting Started

Prerequisites: 

  -Arduino IDE

  -Libraries:
  
  -Wire.h, Servo.h, Adafruit_MPU6050 (or similar)
  
  -QTRSensors.h (for Pololu sensor)

Uploading Code:

  -Connect your Arduino board via USB.
  
  -Open one of the sketches (e.g., Wall_Following, PIDnew) in the Arduino IDE.
  
  -Click Upload to flash the firmware.
  
  -Monitor the Serial output for debugging.
  
  -Press the kill switch to start functioning.
