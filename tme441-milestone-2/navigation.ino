
/*
  TME 441 - Milestone 2
  AC-Team 9

  navigation.ino

  IMPROVED VISUAL ALIGNMENT

  1. Search for the correct AprilTag.
  2. Read heading and X error.
  3. Turn only 60% of the measured angle.
  4. Recheck the HuskyLens.
  5. Repeat until the tag is centered.
  6. Recheck during a long approach.
  7. Drive to the final stopping distance.

  PID and dead reckoning remain unchanged.
*/

// ====================================
// SEARCH SETTINGS
// ====================================

const float SEARCH_TURN_DEG = 15.0f;
const unsigned long SEARCH_TIMEOUT_MS = 90000UL;
const int MAX_SEARCH_TURNS = 24;


// ====================================
// ALIGNMENT SETTINGS
// ====================================

// Accept a maximum of 5 pixels off-center
const float X_TOLERANCE_PX = 5.0f;

// Accept at most 1.5 degrees of heading error
const float HEADING_TOLERANCE_DEG = 1.5f;

// IMPORTANT: Anti-overshoot correction
// Only turn 60% of the measured heading.
const float ALIGN_GAIN = 0.60f;

// Limit each individual correction to 8 degrees
const float MAX_ALIGN_STEP_DEG = 8.0f;

// Prevent endless alignment attempts
const int MAX_ALIGN_ATTEMPTS = 8;

// Wait for the robot and camera to settle
const unsigned long ALIGN_SETTLE_MS = 180UL;


// ====================================
// DISTANCE AND SAFETY SETTINGS
// ====================================

const float MIN_PIVOT_GAP_CM = 15.0f;
const float STOP_GAP_CM = 8.5f;

const float MAX_VALID_GAP_CM = 250.0f;
const float MIN_VALID_GAP_CM = 4.5f;

// Recheck alignment when the robot
// is approximately 40 cm from the tag.
const float RECHECK_GAP_CM = 40.0f;

// Only perform an intermediate movement
// if there is at least 10 cm to travel.
const float RECHECK_MIN_DRIVE_CM = 10.0f;


// ====================================
// ERROR HANDLING
// ====================================

int stopTagWithError(int status) {

  stopDriving();

  Serial.print("NAVIGATION ERROR: ");
  Serial.println(status);

  return status;
}


// ====================================
// CAMERA ALIGNMENT FUNCTION
// ====================================

// Repeatedly correct heading until the tag
// is nearly centered in the camera image.
//
// The TagPose is passed by reference.
// This means it contains the NEW camera
// reading when alignment finishes.

int alignTagToCenter(int tagID, TagPose &tag) {

  for (int attempt = 0;
       attempt < MAX_ALIGN_ATTEMPTS;
       attempt++) {

    float gap = distanceFromFront(tag);

    // Reject unreasonable camera distances
    if (gap <= 0.0f ||
        gap > MAX_VALID_GAP_CM) {

      Serial.println("ERROR: Invalid camera range");
      return stopTagWithError(-3);
    }

    // Calculate horizontal pixel error
    float xError =
        tag.xCenter - CAMERA_CENTER_X;

    Serial.print("ALIGN | Attempt: ");
    Serial.print(attempt + 1);

    Serial.print(" | X Error: ");
    Serial.print(xError, 1);

    Serial.print(" px | Heading: ");
    Serial.print(tag.heading, 2);

    Serial.print(" deg | Gap: ");
    Serial.println(gap, 2);


    // ====================================
    // CHECK WHETHER TAG IS CENTERED
    // ====================================

    if (fabsf(xError) <= X_TOLERANCE_PX &&
        fabsf(tag.heading) <= HEADING_TOLERANCE_DEG) {

      Serial.println("ALIGN | TAG CENTERED");

      return 1;
    }


    // ====================================
    // PREVENT UNSAFE ROTATION
    // ====================================

    if (gap < MIN_PIVOT_GAP_CM) {

      Serial.println("ERROR: Too close to pivot");
      return stopTagWithError(-3);
    }


    // ====================================
    // CALCULATE SMALLER CORRECTION
    // ====================================

    // Original code turned 100% of the
    // estimated heading in a single command.
    //
    // This version starts with 60% to
    // reduce overshoot and then remeasures.

    float turnDeg = ALIGN_GAIN * tag.heading;

    // Limit how far the robot turns at once
    if (turnDeg > MAX_ALIGN_STEP_DEG) {
      turnDeg = MAX_ALIGN_STEP_DEG;
    }

    if (turnDeg < -MAX_ALIGN_STEP_DEG) {
      turnDeg = -MAX_ALIGN_STEP_DEG;
    }

    // Avoid ineffective microscopic turns
    if (fabsf(turnDeg) < 0.15f) {

      Serial.println(
        "ERROR: Alignment not converging"
      );

      return stopTagWithError(-3);
    }

    Serial.print("ALIGN | Commanded turn: ");
    Serial.print(turnDeg, 2);
    Serial.println(" degrees");


    // ====================================
    // ROTATE AND RECHECK CAMERA
    // ====================================

    if (!rotateCCW(turnDeg)) {

      Serial.println("ERROR: Alignment turn failed");
      return stopTagWithError(-3);
    }

    delay(ALIGN_SETTLE_MS);

    // IMPORTANT:
    // Get a NEW camera measurement.
    tag = rangeAndHeadingToTag(tagID);

    if (tag.status == -2) {
      return stopTagWithError(-2);
    }

    if (tag.status != 1) {

      Serial.println("ERROR: Lost tag during alignment");

      return stopTagWithError(-1);
    }
  }


  // Do not drive if alignment never converges
  Serial.println(
    "ERROR: Maximum alignment attempts reached"
  );

  return stopTagWithError(-3);
}


// ====================================
// MAIN TAG NAVIGATION
// ====================================

int driveToTag(int tagID) {

  unsigned long searchStart = millis();
  int searchTurns = 0;

  Serial.print("STARTING SEARCH FOR TAG ");
  Serial.println(tagID);


  // ====================================
  // STEP 1: FIND THE TAG
  // ====================================

  TagPose tag = rangeAndHeadingToTag(tagID);

  while (tag.status == 0 &&
         searchTurns < MAX_SEARCH_TURNS &&
         millis() - searchStart < SEARCH_TIMEOUT_MS) {

    Serial.print("SEARCH | Turn: ");
    Serial.println(searchTurns + 1);

    if (!rotateCCW(SEARCH_TURN_DEG)) {
      return stopTagWithError(-3);
    }

    searchTurns++;

    delay(100);

    tag = rangeAndHeadingToTag(tagID);
  }

  if (tag.status == -2) {
    return stopTagWithError(-2);
  }

  if (tag.status != 1) {
    return stopTagWithError(-1);
  }

  Serial.print("TAG FOUND: ");
  Serial.println(tagID);


  // ====================================
  // STEP 2: CENTER THE TAG
  // ====================================

  int alignStatus = alignTagToCenter(tagID, tag);

  if (alignStatus != 1) {
    return alignStatus;
  }

  float gap = distanceFromFront(tag);

  if (gap <= 0.0f ||
      gap > MAX_VALID_GAP_CM) {

    return stopTagWithError(-3);
  }

  if (gap < MIN_VALID_GAP_CM) {

    Serial.println("ERROR: Already too close");
    return stopTagWithError(-3);
  }


  // ====================================
  // STEP 3: MIDCOURSE CAMERA RECHECK
  // ====================================

  // On a long approach, stop around 40 cm
  // from the tag and correct any new error.
  //
  // This helps if the robot drifts during
  // the first part of its forward movement.

  if (gap > RECHECK_GAP_CM +
            RECHECK_MIN_DRIVE_CM) {

    float firstDriveCm =
        gap - RECHECK_GAP_CM;

    Serial.print("MIDCOURSE | Drive: ");
    Serial.print(firstDriveCm, 2);
    Serial.println(" cm");

    if (!driveDistance(firstDriveCm)) {
      return stopTagWithError(-3);
    }

    stopDriving();

    delay(ALIGN_SETTLE_MS);


    // Get a new camera measurement
    tag = rangeAndHeadingToTag(tagID);

    if (tag.status == -2) {
      return stopTagWithError(-2);
    }

    if (tag.status != 1) {

      Serial.println(
        "ERROR: Tag lost during midcourse check"
      );

      return stopTagWithError(-1);
    }


    // Correct heading again if necessary
    Serial.println("MIDCOURSE | Realigning");

    alignStatus = alignTagToCenter(tagID, tag);

    if (alignStatus != 1) {
      return alignStatus;
    }

    // Use the NEW distance after realignment
    gap = distanceFromFront(tag);
  }


  // ====================================
  // STEP 4: VERIFY FINAL CAMERA RANGE
  // ====================================

  if (gap <= 0.0f ||
      gap > MAX_VALID_GAP_CM ||
      gap < MIN_VALID_GAP_CM) {

    Serial.println("ERROR: Invalid final gap");
    return stopTagWithError(-3);
  }

  Serial.println("===== FINAL APPROACH DATA =====");

  Serial.print("Tag ID: ");
  Serial.println(tagID);

  Serial.print("X Error (px): ");
  Serial.println(
    tag.xCenter - CAMERA_CENTER_X, 1
  );

  Serial.print("Heading (degrees): ");
  Serial.println(tag.heading, 2);

  Serial.print("Front Gap (cm): ");
  Serial.println(gap, 2);

  Serial.println("Robot Position:");
  printPose();


  // ====================================
  // STEP 5: DRIVE TO STOPPING DISTANCE
  // ====================================

  if (gap <= STOP_GAP_CM) {

    Serial.println("Already within stopping gap");

    stopDriving();

    return 1;
  }

  float finalDriveCm =
      gap - STOP_GAP_CM;

  Serial.print("FINAL DRIVE | Distance: ");
  Serial.print(finalDriveCm, 2);
  Serial.println(" cm");

  if (!driveDistance(finalDriveCm)) {
    return stopTagWithError(-3);
  }


  // ====================================
  // STEP 6: STOP AND REPORT
  // ====================================

  stopDriving();

  Serial.print("APPROACH COMPLETE | TAG ");
  Serial.println(tagID);

  printPose();

  return 1;
}
