# ODriveArduino

Arduino library for controlling [ODrive motor controllers](https://odriverobotics.com/).

## Introduction
ODriveArduino is an Arduino library for controlling ODrive motor controllers. ODrive is a high performance, open source motor controller designed for robotics and automation projects.

This code has been modified by @eziron to include additional functions and improvements.

## Installation
To install the ODriveArduino library, follow these steps:
1. Download the latest release from the [GitHub repository](https://github.com/eziron/ODriveArduino).
2. In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`.
3. Select the downloaded ZIP file.

## Getting Started
Here is a basic example to get you started with ODriveArduino. This example demonstrates how to initialize the ODrive and perform basic operations like setting motor positions and reading feedback using an Arduino Leonardo.

## Example Code
### Basic Example
```cpp
#include <ODriveArduino.h>
#include <HardwareSerial.h>

// Set up serial pins to the ODrive
// Arduino Leonardo - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
HardwareSerial& odrive_serial = Serial1;
int baudrate = 115200;

ODriveArduino odrive(odrive_serial);

void setup() {
  odrive_serial.begin(baudrate);
  Serial.begin(115200); // Serial to PC
  
  delay(10);

  Serial.println("Waiting for ODrive...");
  while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  
  Serial.print("DC voltage: ");
  Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
  
  Serial.println("Enabling closed loop control...");
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrive.clearErrors();
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  
  Serial.println("ODrive running!");
}

void loop() {
  float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();
  
  float phase = t * (TWO_PI / SINE_PERIOD);
  
  odrive.SetPosition(0, sin(phase), cos(phase) * (TWO_PI / SINE_PERIOD));
  odrive.SetPosition(1, sin(phase), cos(phase) * (TWO_PI / SINE_PERIOD));

  float velocity_M0, position_M0, velocity_M1, position_M1;
  odrive.GetFeedback(&velocity_M0, &position_M0, &velocity_M1, &position_M1);
  
  Serial.print("pos M0: ");
  Serial.print(position_M0);
  Serial.print(", vel M0: ");
  Serial.print(velocity_M0);
  Serial.print(", pos M1: ");
  Serial.print(position_M1);
  Serial.print(", vel M1: ");
  Serial.print(velocity_M1);
  Serial.println();
}

