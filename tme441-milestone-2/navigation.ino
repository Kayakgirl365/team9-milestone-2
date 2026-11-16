
/*
  TME 441 - Milestone 2
  AC-Team 9

  Find and approach AprilTags using the HuskyLens.

  1. Search for the correct tag.
  2. Measure range AND heading together.
  3. Turn toward the tag.
  4. Confirm alignment and distance.
  5. Drive the calculated distance ONE time.
  6. Stop.

  No ultrasonic sensor.
*/

// Turn 15 degrees when searching for a tag.
const float SEARCH_TURN_DEG = 15.0f;

// Accept a heading error of 4 degrees.
const float HEADING_TOLERANCE_DEG = 4.0f;

// Do not pivot too close to a sign.
const float MIN_PIVOT_GAP_CM = 15.0f;

// Desired stopping distance from the front of robot.
const float STOP_GAP_CM = 8.5f;

// Reject unreasonable camera distances.
const float MAX_VALID_GAP_CM = 250.0f;

// Maximum search time.
const unsigned long SEARCH_TIMEOUT_MS = 90000UL;


// Stop motors if something goes wrong.
int stopTagWithError(int status) {

  stopDriving();

  return status;
}


// Find and approach the specified tag.
//
// Returns:
//  1 = Estimated approach completed
// -1 = Tag not found
// -2 = Camera communication error
// -3 = Motor, encoder, or distance error
int driveToTag(int tagID) {

  unsigned long startTime = millis();
  int searchTurns = 0;

  // ===================================
  // STEP 1: FIND THE TAG
  // ===================================

  // Get the tag's range and heading together.
  TagPose tag = rangeAndHeadingToTag(tagID);

  // Search while the tag is not visible.
  while (tag.status == 0 &&
         searchTurns < 24 &&
         millis() - startTime < SEARCH_TIMEOUT_MS) {

    Serial.print("Searching for Tag ");
    Serial.println(tagID);

    // Rotate 15 degrees to look for the tag.
    if (!rotateCCW(SEARCH_TURN_DEG)) {

      Serial.println("ERROR: Search rotation failed");
      return stopTagWithError(-3);
    }

    searchTurns++;

    // Allow camera image to settle.
    delay(100);

    // Check for the tag again.
    tag = rangeAndHeadingToTag(tagID);
  }

  // Stop if the camera connection failed.
  if (tag.status == -2) {
    return stopTagWithError(-2);
  }

  // Stop if we could not find the correct tag.
  if (tag.status != 1) {
    return stopTagWithError(-1);
  }

  // ===================================
  // STEP 2: MEASURE RANGE AND HEADING
  // ===================================

  Serial.print("Tag found! Range: ");
  Serial.print(distanceFromFront(tag));

  Serial.print(" cm | Heading: ");
  Serial.println(tag.heading);


  // ===================================
  // STEP 3: TURN TOWARD THE TAG
  // ===================================

  // Allow up to two alignment corrections.
  // Each turn uses the camera's measured angle.
  for (int correction = 0; correction < 2; correction++) {

    float gap = distanceFromFront(tag);

    // Reject invalid distance estimates.
    if (gap <= 0.0f || gap > MAX_VALID_GAP_CM) {

      Serial.println("ERROR: Invalid camera range");
      return stopTagWithError(-3);
    }

    // Stop turning when the tag is centered.
    if (fabsf(tag.heading) <= HEADING_TOLERANCE_DEG) {
      break;
    }

    // Avoid pivoting beside the sign.
    if (gap < MIN_PIVOT_GAP_CM) {

      Serial.println("ERROR: Too close to turn");
      return stopTagWithError(-3);
    }

    Serial.print("Turning: ");
    Serial.println(tag.heading);

    // Rotate the angle calculated by the camera.
    // Positive = CCW, negative = CW.
    if (!rotateCCW(tag.heading)) {

      Serial.println("ERROR: Alignment failed");
      return stopTagWithError(-3);
    }

    delay(100);

    // Get a fresh distance and heading after turning.
    tag = rangeAndHeadingToTag(tagID);

    // Stop on camera communication failure.
    if (tag.status == -2) {
      return stopTagWithError(-2);
    }

    // Do not drive if the tag disappeared.
    if (tag.status != 1) {

      Serial.println("ERROR: Lost tag after turning");
      return stopTagWithError(-1);
    }
  }


  // ===================================
  // STEP 4: CHECK FINAL ALIGNMENT
  // ===================================

  // Do not drive if the tag is still off-center.
  if (fabsf(tag.heading) > HEADING_TOLERANCE_DEG) {

    Serial.println("ERROR: Tag not aligned");
    return stopTagWithError(-3);
  }

  // Distance from front of robot to sign.
  float gap = distanceFromFront(tag);

  // Reject invalid distance estimates.
  if (gap <= 0.0f || gap > MAX_VALID_GAP_CM) {

    Serial.println("ERROR: Invalid final distance");
    return stopTagWithError(-3);
  }

  // Stop if the tag is already very close.
  if (gap < 4.5f) {

    Serial.println("ERROR: Tag is too close");
    return stopTagWithError(-3);
  }

  // Already at the estimated stopping distance.
  if (gap <= STOP_GAP_CM) {

    stopDriving();
    return 1;
  }


  // ===================================
  // STEP 5: DRIVE TO THE TAG
  // ===================================

  // Calculate how far to drive.
  float driveCm = gap - STOP_GAP_CM;

  Serial.print("Driving forward: ");
  Serial.print(driveCm);
  Serial.println(" cm");

  // Drive the entire calculated distance ONCE.
  // Your existing PID and slew profile control speed.
  if (!driveDistance(driveCm)) {

    Serial.println("ERROR: Forward movement failed");
    return stopTagWithError(-3);
  }


  // ===================================
  // STEP 6: STOP
  // ===================================

  stopDriving();

  Serial.print("Approach complete for Tag ");
  Serial.println(tagID);

  return 1;
}
