
/*
  TME 441 - Milestone 2 + Dead Reckoning
  AC-Team 9

  Mission:
  1. Find Tag 1
  2. Find Tag 2
  3. Find Tag 3
  4. Return to starting position
  5. Restore starting heading

  Uses updated camera alignment.
*/

#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include <math.h>

HUSKYLENS cam;


// ====================================
// CAMERA DATA STRUCTURE
// ====================================

struct TagPose {

  int status;

  float range;
  float heading;

  int xCenter;
  int height;
};


// ====================================
// FUNCTION DECLARATIONS
// ====================================

TagPose rangeAndHeadingToTag(int tagID);
float distanceFromFront(TagPose tag);

int driveToTag(int tagID);

bool driveDistance(float cm);
bool rotateCCW(float degrees);

void stopDriving();

void resetOdometry();
void updateOdometry();
void printPose();
bool returnHome();


// ====================================
// SETUP
// ====================================

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);

  initMotors();

  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  delay(1000);


  // ====================================
  // INITIALIZE DEAD RECKONING
  // ====================================

  resetOdometry();

  Serial.println("Starting position recorded");

  printPose();


  // ====================================
  // INITIALIZE HUSKYLENS
  // ====================================

  if (!cam.begin(Serial3)) {

    Serial.println("ERROR: HuskyLens not connected");

    stopDriving();
    return;
  }

  Serial.println("HuskyLens connected!");

  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  delay(1000);


  // ====================================
  // TRACK APRILTAGS 1, 2, AND 3
  // ====================================

  for (int tagID = 1; tagID <= 3; tagID++) {

    Serial.print("Searching for Tag ");
    Serial.println(tagID);

    // Uses updated navigation.ino
    int result = driveToTag(tagID);

    stopDriving();

    printPose();

    if (result != 1) {

      Serial.print("Navigation failed: ");
      Serial.println(result);

      return;
    }

    Serial.print("Successfully reached Tag ");
    Serial.println(tagID);

    delay(500);


    // ====================================
    // BACK AWAY AFTER TAGS 1 AND 2
    // ====================================

    if (tagID < 3) {

      Serial.println("Backing away from tag: 9 cm");

      if (!driveDistance(-9.0f)) {

        Serial.println(
          "ERROR: Reverse movement failed"
        );

        stopDriving();
        return;
      }
    }
  }


  // ====================================
  // EXTRA LABOR: RETURN HOME
  // ====================================

  Serial.println(
    "ALL THREE TAGS REACHED - returning home"
  );

  printPose();

  // Back away before turning toward home
  if (!driveDistance(-12.0f)) {

    Serial.println(
      "ERROR: Could not back away from Tag 3"
    );

    stopDriving();
    return;
  }

  if (!returnHome()) {

    Serial.println(
      "ERROR: Return-home movement failed"
    );

    stopDriving();

    printPose();

    return;
  }


  // ====================================
  // FINISH MISSION
  // ====================================

  stopDriving();

  Serial.println(
    "MILESTONE 2 + DEAD RECKONING COMPLETE"
  );

  printPose();
}


// ====================================
// LOOP
// ====================================

void loop() {

  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
}

