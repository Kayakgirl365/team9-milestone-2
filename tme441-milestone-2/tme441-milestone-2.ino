
/*
  TME 441 - Milestone 2 + Extra Labor
  AC-Team 9

  DEAD RECKONING - OPTION 1

  Mission:
  1. Find and approach AprilTag 1.
  2. Find and approach AprilTag 2.
  3. Find and approach AprilTag 3.
  4. Continuously track X, Y, and heading.
  5. Return to starting position.
  6. Face the original starting direction.
  7. Stop permanently.

  Hardware:
  - Arduino Mega
  - HuskyLens camera
  - Two DC motors with encoders
  - Dr. Ford's PID motor controller

  No ultrasonic sensor.
*/

#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include <math.h>


// ====================================
// CAMERA
// ====================================

// HuskyLens connected through Serial3
HUSKYLENS cam;


// ====================================
// APRILTAG DATA STRUCTURE
// ====================================

// Holds range and heading calculated
// from the same camera image.
struct TagPose {

  int status;       // 1 = found, 0 = not found, -2 = error

  float range;      // Distance from camera to tag (cm)

  float heading;    // Direction to tag (degrees)

  int xCenter;      // Tag position in camera image

  int height;       // Tag height in pixels
};


// ====================================
// FUNCTION DECLARATIONS
// ====================================

// Camera functions
TagPose rangeAndHeadingToTag(int tagID);
float distanceFromFront(TagPose p);

// Navigation
int driveToTag(int tagID);

// Driving
bool driveDistance(float centimeters);
bool rotateCCW(float degrees);
void stopDriving();

// Dead reckoning
void resetOdometry();
void updateOdometry();
void printPose();
bool returnHome();


// ====================================
// TEST MODE
// ====================================

// TRUE = Test dead reckoning without tags
// FALSE = Full Milestone 2 and return home

const bool ODOMETRY_TEST_MODE = true;


// ====================================
// SETUP
// ====================================

void setup() {

  // Computer Serial Monitor
  Serial.begin(9600);

  // HuskyLens communication
  // Camera TX -> Mega pin 15
  // Camera RX -> Mega pin 14
  Serial3.begin(9600);

  // Initialize Dr. Ford's motor controller
  initMotors();

  // Start with both motors stopped
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  delay(700);


  // ====================================
  // INITIALIZE DEAD RECKONING
  // ====================================

  // The starting position becomes:
  //
  // X = 0 cm
  // Y = 0 cm
  // Heading = 0 degrees

  resetOdometry();

  Serial.println("Dead reckoning initialized");

  printPose();


  // ====================================
  // DEAD RECKONING TEST MODE
  // ====================================

  if (ODOMETRY_TEST_MODE) {

    Serial.println("Starting odometry test");

    // Drive forward 30 cm
    bool success = driveDistance(30.0f);

    // Turn left 90 degrees
    if (success) {
      success = rotateCCW(90.0f);
    }

    // Drive forward another 30 cm
    if (success) {
      success = driveDistance(30.0f);
    }

    // Display estimated position
    printPose();

    // Return to original location and heading
    if (success) {

      Serial.println("Returning to starting position");

      success = returnHome();
    }

    stopDriving();

    if (success) {
      Serial.println("ODOMETRY TEST COMPLETE");
    } else {
      Serial.println("ODOMETRY TEST FAILED");
    }

    // Do not run tag navigation in test mode
    return;
  }


  // ====================================
  // CONNECT HUSKYLENS
  // ====================================

  if (!cam.begin(Serial3)) {

    Serial.println("ERROR: HuskyLens not connected");

    stopDriving();
    return;
  }

  Serial.println("HuskyLens connected");

  // Select AprilTag recognition mode
  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  delay(700);


  // ====================================
  // MILESTONE 2 - FIND TAGS 1, 2, 3
  // ====================================

  for (int tagID = 1; tagID <= 3; tagID++) {

    Serial.print("Searching for Tag ");
    Serial.println(tagID);

    // Uses the team's navigation.ino
    //
    // 1. Search for the tag
    // 2. Measure range and heading
    // 3. Turn toward the tag
    // 4. Drive calculated distance once
    // 5. Stop at estimated target location

    int result = driveToTag(tagID);

    // Stop after approaching each tag
    stopDriving();

    // Display updated X, Y, and heading
    printPose();


    // ====================================
    // CHECK NAVIGATION RESULT
    // ====================================

    if (result != 1) {

      Serial.print("Navigation failed: ");
      Serial.println(result);

      // Do not continue after a failure
      return;
    }

    Serial.print("Successfully reached Tag ");
    Serial.println(tagID);

    delay(500);


    // ====================================
    // BACK AWAY AFTER TAGS 1 AND 2
    // ====================================

    // Give robot space to rotate toward
    // the next tag.

    if (tagID < 3) {

      Serial.println("Backing away 9 cm");

      if (!driveDistance(-9.0f)) {

        Serial.println("ERROR: Reverse movement failed");

        stopDriving();
        return;
      }
    }
  }


  // ====================================
  // EXTRA LABOR - RETURN HOME
  // ====================================

  Serial.println("All three tags completed!");

  // Display location after Tag 3
  printPose();

  // Back away before turning toward home
  // Ensure the space behind robot is clear.
  Serial.println("Backing away from Tag 3");

  if (!driveDistance(-12.0f)) {

    Serial.println("ERROR: Cannot back away");

    stopDriving();
    return;
  }


  // Calculate direction and distance home
  // using the stored encoder-based position.

  Serial.println("Returning to starting position");

  bool homeSuccess = returnHome();


  // ====================================
  // MISSION COMPLETE
  // ====================================

  stopDriving();

  if (homeSuccess) {

    Serial.println("RETURN HOME COMPLETE");

    Serial.println("Starting position estimated");
    Serial.println("Original heading restored");

  } else {

    Serial.println("ERROR: Return home failed");
  }

  // Show final estimated X, Y, theta
  printPose();

  Serial.println("MILESTONE 2 FINISHED");
}


// ====================================
// LOOP
// ====================================

void loop() {

  // Mission runs only once.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
}
