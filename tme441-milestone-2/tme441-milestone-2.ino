
/*
  TME 441 - Milestone 2
  AC-Team 9

  Authors: Alex Brown, Danielle Platt, Nathan Huynh

  Goal:
  1. Find AprilTag 1 and drive toward it.
  2. Stop near Tag 1.
  3. Find and approach Tag 2.
  4. Find and approach Tag 3.
  5. Stop permanently.

  Uses HuskyLens camera and PID motor control.
  No ultrasonic sensor or dead reckoning.
*/

#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include <math.h>

// Create the HuskyLens camera object
HUSKYLENS cam;

// Store the camera measurements for one tag
struct TagPose {
  int status;       // 1 = found, 0 = not found, -2 = error
  float range;      // Estimated distance in cm
  float heading;    // Angle toward tag in degrees
  int xCenter;      // Tag's horizontal position
  int height;       // Tag's height in pixels
};

// Functions defined in the other Arduino tabs
TagPose rangeAndHeadingToTag(int tagID);
float distanceFromFront(TagPose tag);
int driveToTag(int tagID);
bool driveDistance(float cm);
bool rotateCCW(float degrees);


// ====================================
// SETUP - RUNS ONCE
// ====================================

void setup() {

  // Start Serial Monitor at 9600 baud
  Serial.begin(9600);

  // HuskyLens connected to Arduino Mega Serial3
  // Camera TX -> Arduino pin 15
  // Camera RX -> Arduino pin 14
  Serial3.begin(9600);

  // Initialize the motor pins
  initMotors();

  // Make sure motors start stopped
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  // Allow the camera to start
  delay(1000);

  // Connect to HuskyLens
  if (!cam.begin(Serial3)) {

    Serial.println("ERROR: HuskyLens not connected");

    // Do not move if camera connection fails
    return;
  }

  Serial.println("HuskyLens connected!");

  // Set camera to AprilTag recognition mode
  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  delay(1000);

  // ====================================
  // FIND TAGS 1, 2, AND 3
  // ====================================

  for (int tagID = 1; tagID <= 3; tagID++) {

    Serial.print("Searching for Tag ");
    Serial.println(tagID);

    // Search for the correct tag
    // Rotate until detected
    // Align using camera heading
    // Drive toward the tag using encoder feedback
    // Stop at the estimated target distance
    int result = driveToTag(tagID);

    // Stop motors after each tag
    motorHalt(LMOTOR);
    motorHalt(RMOTOR);

    // Check if navigation succeeded
    if (result != 1) {

      Serial.print("Navigation failed: ");
      Serial.println(result);

      // Stop the entire mission if an error occurs
      return;
    }

    Serial.print("Successfully reached Tag ");
    Serial.println(tagID);

    // Pause briefly at each tag
    delay(500);

    // After Tag 1 and Tag 2, back up 9 cm
    // This provides room to rotate toward the next tag
    // Make sure the space behind the robot is clear
    if (tagID < 3) {

      Serial.println("Backing away from tag");

      // Negative distance means drive backward
      if (!driveDistance(-9.0f)) {

        Serial.println("ERROR: Reverse movement failed");

        motorHalt(LMOTOR);
        motorHalt(RMOTOR);

        return;
      }
    }
  }

  // ====================================
  // MISSION COMPLETE
  // ====================================

  // Stop permanently after Tag 3
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  Serial.println("MILESTONE 2 COMPLETE!");
}


// ====================================
// LOOP - DO NOT REPEAT THE MISSION
// ====================================

void loop() {

  // Keep both motors stopped
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  // Do not restart the tag sequence
}
