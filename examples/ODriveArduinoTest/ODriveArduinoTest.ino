/**
 * @file ODriveArduinoTest.ino
 * @author Your Name (@eziron1)
 * @brief Example sketch for the ODriveArduino library with optimized functions.
 * 
 * This sketch provides an interactive serial menu to test the various features
 * of the ODrive, showcasing the library's capabilities, especially the 
 * optimized functions for high-frequency communication.
 * 
 * --- MENU COMMANDS ---
 * 'h': Print this help menu
 * '0', '1': Calibrate motor 0 or 1
 * 's': Run a sinusoidal position move on both motors
 * 'v': Run a velocity control test on motor 0
 * 't': Run a trapezoidal move test on motor 0
 * 'c': Run a current (torque) control test on motor 0
 * 'o': Run an OPTIMIZED high-frequency feedback loop test
 * 'b': Read ODrive bus voltage
 * 'e': Check for errors on both axes
 * 'r': Reboot the ODrive
 * 
 */

// Includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "ODriveArduino.h"

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX (GPIO2)
// pin 1: TX - connect to ODrive RX (GPIO1)
HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX (GPIO2)
// pin 18: TX - connect to ODrive RX (GPIO1)
// HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this can be unreliable.
// pin 8: RX - connect to ODrive TX (GPIO2)
// pin 9: TX - connect to ODrive RX (GPIO1)
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);

void printMenu();

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino Library Test Sketch");
  Serial.println("Setting parameters...");

  // Set parameters using the library's functions
  for (int axis = 0; axis < 2; ++axis) {
    odrive.WriteProperty(axis, "controller.config.vel_limit", 20.0f);
    odrive.WriteProperty(axis, "motor.config.current_lim", 25.0f);
    odrive.WriteProperty(axis, "motor.config.requested_current_range", 30.0f); // Recommended for gimbal motors
  }

  Serial.println("Ready! Use the menu below.");
  printMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'h':
        printMenu();
        break;
      case '0':
      case '1':
        calibrateMotor(c - '0');
        break;
      case 's':
        testSinusoidalMove();
        break;
      case 'v':
        testVelocityControl();
        break;
      case 't':
        testTrapezoidalMove();
        break;
      case 'c':
        testCurrentControl();
        break;
      case 'o':
        testOptimizedFeedbackLoop();
        break;
      case 'b':
        readBusVoltage();
        break;
      case 'e':
        checkErrors();
        break;
      case 'r':
        Serial.println("Rebooting ODrive...");
        odrive.Reboot();
        break;
    }
  }
}

void printMenu() {
  Serial.println("\n--- ODrive Test Menu ---");
  Serial.println("'h': Print this help menu");
  Serial.println("'0', '1': Calibrate motor 0 or 1");
  Serial.println("'s': Run sinusoidal position move");
  Serial.println("'v': Run velocity control test (M0)");
  Serial.println("'t': Run trapezoidal move test (M0)");
  Serial.println("'c': Run current (torque) control test (M0)");
  Serial.println("'o': Run OPTIMIZED feedback loop test");
  Serial.println("'b': Read bus voltage");
  Serial.println("'e': Check for errors");
  Serial.println("'r': Reboot the ODrive\n");
}

void calibrateMotor(int motor_num) {
  Serial.print("Calibrating Axis");
  Serial.print(motor_num);
  Serial.println("...");
  
  if (!odrive.run_state(motor_num, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true, 25.0f)) {
    Serial.println("ERROR: Calibration failed!");
    return;
  }
  
  // Optional: Put axis into closed loop control after calibration
  odrive.run_state(motor_num, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  Serial.print("Axis");
  Serial.print(motor_num);
  Serial.println(" calibrated and in Closed Loop Control.");
}

void testSinusoidalMove() {
  Serial.println("Executing sinusoidal test move for 5 seconds...");
  long start_time = millis();
  while (millis() - start_time < 5000) {
    float phase = (millis() - start_time) / 1000.0f * 2.0f * M_PI;
    float pos_m0 = 2.0f * cos(phase);
    float pos_m1 = 2.0f * sin(phase);
    odrive.SetPosition(0, pos_m0);
    odrive.SetPosition(1, pos_m1);
    delay(10);
  }
  Serial.println("Move complete.");
}

void testVelocityControl() {
  Serial.println("Testing velocity control on Motor 0...");
  Serial.println("Ramping up to 5 turns/sec...");
  odrive.SetVelocity(0, 5.0f);
  delay(2000);
  Serial.println("Ramping down to -5 turns/sec...");
  odrive.SetVelocity(0, -5.0f);
  delay(2000);
  Serial.println("Setting velocity to 0.");
  odrive.SetVelocity(0, 0.0f);
  Serial.println("Test complete.");
}

void testTrapezoidalMove() {
  Serial.println("Testing trapezoidal move on Motor 0 to position 5...");
  odrive.TrapezoidalMove(0, 5.0f);
  Serial.println("Command sent. Waiting for move to complete...");
  odrive.WaitIdle(0, 10.0f); // Wait for motor 0 to be idle
  
  Serial.print("Move complete. Final position: ");
  Serial.println(odrive.GetPosition(0));
}

void testCurrentControl() {
  Serial.println("Testing current (torque) control on Motor 0.");
  Serial.println("Applying 0.5A of current for 3 seconds. Feel the torque!");
  odrive.SetCurrent(0, 0.5f);
  delay(3000);
  Serial.println("Releasing torque.");
  odrive.SetCurrent(0, 0.0f);
  Serial.println("Test complete.");
}

void testOptimizedFeedbackLoop() {
  Serial.println("\n--- Testing OPTIMIZED Feedback Loop ---");
  Serial.println("This test sends commands to both motors and reads all feedback in a single transaction.");
  Serial.println("Running for 5 seconds...\n");

  long start_time = millis();
  int iterations = 0;
  float vbus, vel0, pos0, vel1, pos1;

  while (millis() - start_time < 5000) {
    // Generate some simple velocity commands
    float set_v0 = 2.0f * sin((millis() - start_time) / 1000.0f * M_PI);
    float set_v1 = 2.0f * cos((millis() - start_time) / 1000.0f * M_PI);

    // This is the optimized function call!
    odrive.SetVelocityBoth_GetFeedback_Vbus(set_v0, set_v1, &vbus, &vel0, &pos0, &vel1, &pos1);
    
    iterations++;
    // Optional: Print feedback occasionally to not flood the serial port
    if (iterations % 50 == 0) {
        Serial.print("Vbus: ");
        Serial.print(vbus);
        Serial.print("V, M0_pos: ");
        Serial.print(pos0);
        Serial.print(", M0_vel: ");
        Serial.print(vel0);
        Serial.print(", M1_pos: ");
        Serial.print(pos1);
        Serial.print(", M1_vel: ");
        Serial.println(vel1);
    }
    delay(10); // Simulate other work in the loop
  }
  
  // Stop motors after test
  odrive.SetVelocityBoth(0.0f, 0.0f);

  long duration = millis() - start_time;
  Serial.println();
  Serial.print("Test complete. Ran ");
  Serial.print(iterations);
  Serial.print(" loops in ");
  Serial.print(duration);
  Serial.println(" ms.");
  Serial.print(" Average loop frequency: ");
  Serial.print((float)iterations / (duration / 1000.0f));
  Serial.println(" Hz");
}

void readBusVoltage() {
  float vbus = 0.0f;
  // Use the library's property reading function
  odrive.ReadProperty(0, "vbus_voltage", &vbus);
  Serial.print("Vbus voltage: ");
  Serial.print(vbus);
  Serial.println(" V");
}

void checkErrors() {
  Serial.println("Checking for errors...");
  for (int axis = 0; axis < 2; ++axis) {
    int error = 0;
    odrive.ReadProperty(axis, "error", &error);
    if (error != 0) {
      Serial.print("ERROR on Axis");
      Serial.print(axis);
      Serial.print("! Error code: 0x");
      Serial.println(String(error, HEX));
    } else {
      Serial.print("Axis");
      Serial.print(axis);
      Serial.println(": No errors.");
    }
  }
  Serial.println("See ODrive documentation for error code meanings.");
}