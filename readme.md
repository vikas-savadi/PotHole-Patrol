# PotHole Patrol 

PotHole Patrol is a rover-based embedded system designed to **detect, report, and track potholes** on Karnataka roads in real time. The rover uses onboard sensors to measure road irregularities and transmits location-tagged pothole data over WiFi.
---
Not only potholes, but **can also detect and speedbreakers ** 

---
##  Features

* GPS-based pothole location tracking
* Real-time wireless reporting
* Ultrasonic pothole depth sensing
* Accelerometer-based vibration detection
* Map-based visualization
* Photo upload capability
* Notification support

---
## Hardware Used

* ESP8266 WiFi module — wireless communication
* HC-SR04 ultrasonic sensor — pothole depth detection
* NEO-6M GPS module — location tracking
* MPU6050 accelerometer — vibration/motion sensing
* Rover/car chassis
* Breadboard & jumper wires
* Power supply
---

## Software Requirements

* Arduino IDE
* ESP8266 board package
* Required libraries:
  * TinyGPS++
  * MPU6050 library
  * ESP8266WiFi
---

## Setup Instructions

1. Assemble hardware according to the wiring diagram

2. Install Arduino IDE and required libraries

3. Clone this repository:

   git clone [https://github.com/vikas-savadi/PotHole-Patrol.git](https://github.com/vikas-savadi/PotHole-Patrol.git)

4. Open the project in Arduino IDE

5. Upload code to ESP8266

6. Power the rover and monitor serial output

---

## Project Goal

To provide an automated and scalable system for **road maintenance monitoring**, helping authorities detect potholes early and improve road safety.

---
