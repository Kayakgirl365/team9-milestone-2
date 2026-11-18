
/*
  TME 441 - Milestone 2
  AC-Team 9

  navigation.ino

  Find AprilTags 1, 2, and 3.

  Search -> Detect -> Calculate heading ->
  Align -> Drive calculated distance -> Stop.

  Includes Serial Monitor debugging.
*/

// ====================================
// NAVIGATION SETTINGS
// ====================================

const float SEARCH_TURN_DEG = 15.0f;

const float HEADING_TOLERANCE_DEG = 4.0f;

const float MIN_PIVOT_GAP_CM = 15.0f;

const float STOP_GAP_CM = 8.5f;

const float MAX_VALID_GAP_CM = 250.0f;

const unsigned long SEARCH_TIMEOUT_MS = 90000UL;


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
// DRIVE TO SPECIFIED TAG
// ====================================

int driveToTag(int tagID) {

  unsigned long startTime = millis();
  int searchTurns = 0;

  Serial.print("STARTING SEARCH FOR TAG ");
  Serial.println(tagID);


  // ====================================
  // STEP 1: FIND THE TAG
  // ====================================

  TagPose tag = rangeAndHeadingToTag(tagID);

  while (tag.status == 0 &&
         searchTurns < 24 &&
         millis() - startTime < SEARCH_TIMEOUT_MS) {

    Serial.print("Search rotation ");
    Serial.print(searchTurns + 1);
    Serial.println(" | Turning 15 degrees");

    if (!rotateCCW(SEARCH_TURN_DEG)) {

      Serial.println("ERROR: Search rotation failed");
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
    Serial.println("ERROR: Tag not found");
    return stopTagWithError(-1);
  }


  // ====================================
  // STEP 2: TAG DETECTED
  // ====================================

  Serial.print("TAG FOUND: ");
  Serial.println(tagID);

  Serial.print("Initial Distance (cm): ");
  Serial.println(distanceFromFront(tag), 2);

  Serial.print("Initial Heading (deg): ");
  Serial.println(tag.heading, 2);


  // ====================================
  // STEP 3: ALIGN WITH THE TAG
  // ====================================

  for (int correction = 0; correction < 2; correction++) {

    float gap = distanceFromFront(tag);

    if (gap <= 0.0f || gap > MAX_VALID_GAP_CM) {

      Serial.println("ERROR: Invalid camera range");
      return stopTagWithError(-3);
    }

    // Stop turning once heading is acceptable.
    if (fabsf(tag.heading) <= HEADING_TOLERANCE_DEG) {

      Serial.println("Tag alignment acceptable");
      break;
    }

    if (gap < MIN_PIVOT_GAP_CM) {

      Serial.println("ERROR: Too close to turn");
      return stopTagWithError(-3);
    }

    Serial.print("Alignment Correction (deg): ");
    Serial.println(tag.heading, 2);

    // Turn toward the tag.
    if (!rotateCCW(tag.heading)) {

      Serial.println("ERROR: Alignment failed");
      return stopTagWithError(-3);
    }

    delay(100);

    // Recheck camera after turning.
    tag = rangeAndHeadingToTag(tagID);

    if (tag.status == -2) {
      return stopTagWithError(-2);
    }

    if (tag.status != 1) {

      Serial.println("ERROR: Lost tag after turning");
      return stopTagWithError(-1);
    }
  }


  // ====================================
  // STEP 4: VERIFY ALIGNMENT
  // ====================================

  if (fabsf(tag.heading) > HEADING_TOLERANCE_DEG) {

    Serial.println("ERROR: Heading outside tolerance");
    return stopTagWithError(-3);
  }

  float gap = distanceFromFront(tag);

  if (gap <= 0.0f || gap > MAX_VALID_GAP_CM) {

    Serial.println("ERROR: Invalid final distance");
    return stopTagWithError(-3);
  }

  if (gap < 4.5f) {

    Serial.println("ERROR: Tag is too close");
    return stopTagWithError(-3);
  }


  // ====================================
  // STEP 5: CALCULATE APPROACH
  // ====================================

  Serial.println("===== FINAL CAMERA MEASUREMENT =====");

  Serial.print("Target Tag ID: ");
  Serial.println(tagID);

  Serial.print("Estimated Front Gap (cm): ");
  Serial.println(gap, 2);

  Serial.print("Final Heading Error (deg): ");
  Serial.println(tag.heading, 2);

  Serial.println("Robot Position Before Approach:");
  printPose();

  // Already at desired stopping distance.
  if (gap <= STOP_GAP_CM) {

    stopDriving();
    return 1;
  }

  float driveCm = gap - STOP_GAP_CM;

  Serial.print("Calculated Driving Distance (cm): ");
  Serial.println(driveCm, 2);

  Serial.println("====================================");


  // ====================================
  // STEP 6: DRIVE ONCE TO TARGET
  // ====================================

  if (!driveDistance(driveCm)) {

    Serial.println("ERROR: Forward movement failed");
    return stopTagWithError(-3);
  }

  stopDriving();


  // ====================================
  // STEP 7: PRINT FINAL POSITION
  // ====================================

  Serial.print("APPROACH COMPLETE | TAG ");
  Serial.println(tagID);

  Serial.println("Robot Position After Approach:");
  printPose();

  return 1;
}
