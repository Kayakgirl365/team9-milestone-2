
/*
  TME441 Milestone 2: live camera-guided approach
  AC-Team 9

  - Initial tag search and alignment are retained.
  - HuskyLens is polled about every 140 ms while driving.
  - Differential wheel speeds correct heading continuously.
  - Final 25 cm uses encoder feedback without stopping.
  - PID and dead reckoning remain active.

  IMPORTANT: No obstacle avoidance.
  Test slowly and supervise the robot.
*/

// ====================================
// SEARCH AND ALIGNMENT
// ====================================

const float SEARCH_TURN_DEG = 15.0f;
const unsigned long SEARCH_TIMEOUT_MS = 90000UL;
const int MAX_SEARCH_TURNS = 24;

const float X_TOLERANCE_PX = 5.0f;
const float HEADING_TOLERANCE_DEG = 1.5f;

const float ALIGN_GAIN = 0.60f;
const float MAX_ALIGN_STEP_DEG = 8.0f;
const int MAX_ALIGN_ATTEMPTS = 8;
const unsigned long ALIGN_SETTLE_MS = 180UL;

const float MIN_PIVOT_GAP_CM = 15.0f;


// ====================================
// DISTANCE AND SAFETY SETTINGS
// ====================================

const float STOP_GAP_CM = 8.5f;
const float MIN_VALID_GAP_CM = 4.5f;
const float MAX_VALID_GAP_CM = 250.0f;

const float MAX_LIVE_TRAVEL_CM = 260.0f;

// Switch to encoder-only finish before
// the tag becomes too large for the image.
const float VISION_HANDOFF_GAP_CM = 25.0f;


// ====================================
// CONTINUOUS STEERING SETTINGS
// ====================================

const unsigned long CAMERA_POLL_MS = 140UL;
const unsigned long CAMERA_LOST_TIMEOUT_MS = 350UL;
const unsigned long LIVE_REPORT_MS = 300UL;

// Steering correction in ticks/sec per degree.
const float LIVE_STEER_GAIN = 18.0f;

const long LIVE_MAX_STEER_TICKS = 260L;
const long LIVE_MIN_WHEEL_TICKS = 120L;

// Reduce to 450 for initial physical testing.
const long LIVE_MAX_BASE_TICKS = 900L;

// Smooth acceleration and deceleration.
const float LIVE_SPEED_PER_CM = 19.0f;
const long LIVE_ACCEL_STEP_TICKS = 30L;
const long LIVE_DECEL_STEP_TICKS = 60L;


// ====================================
// ERROR HANDLING
// ====================================

int stopTagWithError(int status) {

  stopDriving();

  Serial.print(F("NAVIGATION ERROR: "));
  Serial.println(status);

  return status;
}


// ====================================
// INITIAL CAMERA ALIGNMENT
// ====================================

// Rotate carefully before beginning
// the continuous forward approach.

int alignTagToCenter(int tagID, TagPose &tag) {

  for (int attempt = 0;
       attempt < MAX_ALIGN_ATTEMPTS;
       attempt++) {

    float gap = distanceFromFront(tag);

    if (gap <= 0.0f ||
        gap > MAX_VALID_GAP_CM) {
      return stopTagWithError(-3);
    }

    float xError = tag.xCenter - CAMERA_CENTER_X;

    Serial.print(F("ALIGN | x_px="));
    Serial.print(xError, 1);

    Serial.print(F(" | bearing_deg="));
    Serial.println(tag.heading, 2);

    // Accept alignment if sufficiently centered.
    if (fabsf(xError) <= X_TOLERANCE_PX &&
        fabsf(tag.heading) <= HEADING_TOLERANCE_DEG) {

      Serial.println(F("ALIGN | ready to drive"));
      return 1;
    }

    // Avoid pivoting too close to the sign.
    if (gap < MIN_PIVOT_GAP_CM) {

      Serial.println(F("ERROR: Too close to pivot"));
      return stopTagWithError(-3);
    }

    // Use 60% of measured angle to reduce
    // the risk of turn overshoot.
    float turnDeg = ALIGN_GAIN * tag.heading;

    if (turnDeg > MAX_ALIGN_STEP_DEG)
      turnDeg = MAX_ALIGN_STEP_DEG;

    if (turnDeg < -MAX_ALIGN_STEP_DEG)
      turnDeg = -MAX_ALIGN_STEP_DEG;

    if (fabsf(turnDeg) < 0.15f)
      return stopTagWithError(-3);

    // Rotate, settle, and get a new reading.
    if (!rotateCCW(turnDeg))
      return stopTagWithError(-3);

    delay(ALIGN_SETTLE_MS);

    tag = rangeAndHeadingToTag(tagID);

    if (tag.status == -2)
      return stopTagWithError(-2);

    if (tag.status != 1)
      return stopTagWithError(-1);
  }

  Serial.println(
    F("ERROR: Initial alignment did not converge")
  );

  return stopTagWithError(-3);
}


// ====================================
// CONTINUOUS CAMERA-GUIDED DRIVING
// ====================================

// Camera measurements are taken during movement.
// Both wheel speeds are adjusted without stopping.

int liveDriveToTag(int tagID, TagPose &tag) {

  if (tag.status != 1)
    return stopTagWithError(-1);

  float gap = distanceFromFront(tag);

  if (gap < MIN_VALID_GAP_CM ||
      gap > MAX_VALID_GAP_CM) {
    return stopTagWithError(-3);
  }

  if (gap <= STOP_GAP_CM) {
    stopDriving();
    return 1;
  }

  // Begin one continuous forward approach.
  stopDriving();
  resetPID();

  const unsigned long startTime = millis();

  unsigned long lastCameraPoll =
      startTime - CAMERA_POLL_MS;

  unsigned long lastValidCamera = startTime;
  unsigned long lastReport = startTime;

  unsigned long lastLeftMove = startTime;
  unsigned long lastRightMove = startTime;

  long lastLeftTicks = enc_l.read();
  long lastRightTicks = enc_r.read();

  long baseSpeed = MIN_SPEED_TICKS;

  bool currentlyVisible = true;
  bool encoderFinish = false;

  float handoffTravelCm = 0.0f;
  float handoffRemainingCm = 0.0f;

  float traveledCm = 0.0f;

  float ticksToCm =
      (PI * WHEEL_DIAMETER_CM) /
      ENCODER_TICKS_PER_WHEEL_REV;

  Serial.print(F("LIVE | Starting tag "));
  Serial.println(tagID);


  // ====================================
  // MAIN MOVEMENT LOOP
  // ====================================

  while (true) {

    // Keep dead reckoning updated.
    updateOdometry();

    unsigned long now = millis();


    // ====================================
    // READ ENCODERS
    // ====================================

    long leftTicks = enc_l.read();
    long rightTicks = enc_r.read();

    long leftDelta = leftTicks - lastLeftTicks;
    long rightDelta = rightTicks - lastRightTicks;

    if (leftDelta != 0)
      lastLeftMove = now;

    if (rightDelta != 0)
      lastRightMove = now;

    traveledCm +=
        0.5f * (labs(leftDelta) + labs(rightDelta))
        * ticksToCm;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;


    // ====================================
    // SAFETY TIMEOUTS
    // ====================================

    if (now - startTime > MOVEMENT_TIMEOUT_MS ||
        now - lastLeftMove > NO_PROGRESS_TIMEOUT_MS ||
        now - lastRightMove > NO_PROGRESS_TIMEOUT_MS ||
        traveledCm > MAX_LIVE_TRAVEL_CM) {

      Serial.println(
        F("ERROR: live approach timeout / stall / travel limit")
      );

      return stopTagWithError(-3);
    }


    // ====================================
    // READ CAMERA WHILE MOVING
    // ====================================

    if (!encoderFinish &&
        now - lastCameraPoll >= CAMERA_POLL_MS) {

      lastCameraPoll = now;

      // Motors have NOT been stopped.
      TagPose newest = rangeAndHeadingToTag(tagID);

      // Record wheel movement during camera read.
      updateOdometry();

      now = millis();

      if (newest.status == -2) {

        Serial.println(
          F("ERROR: Camera communication lost while moving")
        );

        return stopTagWithError(-2);
      }

      if (newest.status == 1) {

        float newestGap = distanceFromFront(newest);

        if (newestGap < MIN_VALID_GAP_CM ||
            newestGap > MAX_VALID_GAP_CM) {

          Serial.println(
            F("ERROR: Invalid live camera distance")
          );

          return stopTagWithError(-3);
        }

        tag = newest;
        gap = newestGap;

        currentlyVisible = true;
        lastValidCamera = now;


        // Stop if estimated gap is reached.
        if (gap <= STOP_GAP_CM) {

          stopDriving();

          Serial.println(
            F("LIVE | Tag reached by camera measurement")
          );

          printPose();

          return 1;
        }


        // ====================================
        // SWITCH TO ENCODER FINISH
        // ====================================

        if (gap <= VISION_HANDOFF_GAP_CM) {

          long handoffL = enc_l.read();
          long handoffR = enc_r.read();

          traveledCm +=
              0.5f * (
                labs(handoffL - lastLeftTicks) +
                labs(handoffR - lastRightTicks)
              ) * ticksToCm;

          lastLeftTicks = handoffL;
          lastRightTicks = handoffR;

          encoderFinish = true;

          handoffRemainingCm = gap - STOP_GAP_CM;
          handoffTravelCm = traveledCm;

          Serial.print(
            F("LIVE | HANDOFF TO ENCODERS | cm=")
          );

          Serial.println(handoffRemainingCm, 2);
        }

      } else {

        // One missed frame does not stop motors.
        currentlyVisible = false;
      }
    }


    // ====================================
    // CAMERA LOSS SAFETY
    // ====================================

    if (!encoderFinish &&
        now - lastValidCamera >
        CAMERA_LOST_TIMEOUT_MS) {

      Serial.println(
        F("ERROR: Tag lost for too long while moving")
      );

      return stopTagWithError(-1);
    }


    // ====================================
    // CALCULATE REMAINING DISTANCE
    // ====================================

    float remaining = encoderFinish
        ? handoffRemainingCm -
          (traveledCm - handoffTravelCm)
        : gap - STOP_GAP_CM;

    if (encoderFinish && remaining <= 0.0f) {

      stopDriving();

      Serial.println(
        F("LIVE | Tag reached by encoder finish")
      );

      printPose();

      return 1;
    }

    if (remaining < 0.0f)
      remaining = 0.0f;


    // ====================================
    // CALCULATE FORWARD SPEED
    // ====================================

    long desiredBase =
        MIN_SPEED_TICKS +
        (long)(LIVE_SPEED_PER_CM * remaining);

    if (desiredBase > LIVE_MAX_BASE_TICKS)
      desiredBase = LIVE_MAX_BASE_TICKS;

    // Slow if camera temporarily loses the tag.
    if ((!currentlyVisible && !encoderFinish) ||
        remaining <= 1.0f) {

      desiredBase = MIN_SPEED_TICKS;
    }

    // Smoothly accelerate or decelerate.
    if (baseSpeed < desiredBase) {

      baseSpeed += LIVE_ACCEL_STEP_TICKS;

      if (baseSpeed > desiredBase)
        baseSpeed = desiredBase;

    } else if (baseSpeed > desiredBase) {

      baseSpeed -= LIVE_DECEL_STEP_TICKS;

      if (baseSpeed < desiredBase)
        baseSpeed = desiredBase;
    }


    // ====================================
    // CONTINUOUS HEADING CORRECTION
    // ====================================

    float xError =
        tag.xCenter - CAMERA_CENTER_X;

    // Steering strength based on camera angle.
    float steerFloat =
        LIVE_STEER_GAIN * tag.heading;

    // Do not steer using stale or
    // sufficiently centered measurements.
    if (encoderFinish ||
        !currentlyVisible ||
        (fabsf(xError) <= X_TOLERANCE_PX &&
         fabsf(tag.heading) <= HEADING_TOLERANCE_DEG)) {

      steerFloat = 0.0f;
    }

    long steer = (long)steerFloat;

    // Limit maximum steering correction.
    if (steer > LIVE_MAX_STEER_TICKS)
      steer = LIVE_MAX_STEER_TICKS;

    if (steer < -LIVE_MAX_STEER_TICKS)
      steer = -LIVE_MAX_STEER_TICKS;


    // ====================================
    // MAINTAIN FORWARD MOTION
    // ====================================

    // Prevent either wheel from reversing
    // during the camera-guided approach.
    long allowedSteer =
        baseSpeed - LIVE_MIN_WHEEL_TICKS;

    if (steer > allowedSteer)
      steer = allowedSteer;

    if (steer < -allowedSteer)
      steer = -allowedSteer;


    // ====================================
    // DIFFERENTIAL STEERING
    // ====================================

    // Tag left:
    //   right wheel faster, left wheel slower.
    //
    // Tag right:
    //   left wheel faster, right wheel slower.

    long leftCommand = baseSpeed - steer;
    long rightCommand = baseSpeed + steer;

    if (leftCommand > MAX_SPEED_TICKS)
      leftCommand = MAX_SPEED_TICKS;

    if (rightCommand > MAX_SPEED_TICKS)
      rightCommand = MAX_SPEED_TICKS;


    // Dr. Ford's PID controller handles
    // the actual PWM motor commands.
    updatePID(leftCommand, rightCommand, false);


    // ====================================
    // SERIAL DEBUG INFORMATION
    // ====================================

    if (now - lastReport >= LIVE_REPORT_MS) {

      lastReport = now;

      Serial.print(F("LIVE | mode="));
      Serial.print(
        encoderFinish ? F("ENCODER") : F("CAMERA")
      );

      Serial.print(F(" | remaining_cm="));
      Serial.print(remaining, 1);

      Serial.print(F(" | gap_cm="));
      Serial.print(gap, 1);

      Serial.print(F(" | x_error_px="));
      Serial.print(xError, 1);

      Serial.print(F(" | bearing_deg="));
      Serial.print(tag.heading, 2);

      Serial.print(F(" | left_tps="));
      Serial.print(leftCommand);

      Serial.print(F(" | right_tps="));
      Serial.println(rightCommand);
    }

    // Approximately 20 ms PID update interval.
    delay(20);
  }
}


// ====================================
// FIND AND APPROACH APRILTAG
// ====================================

int driveToTag(int tagID) {

  unsigned long startTime = millis();
  int searchTurns = 0;

  Serial.print(F("SEARCH | Tag "));
  Serial.println(tagID);

  TagPose tag = rangeAndHeadingToTag(tagID);


  // Search until correct tag is visible.
  while (tag.status == 0 &&
         searchTurns < MAX_SEARCH_TURNS &&
         millis() - startTime < SEARCH_TIMEOUT_MS) {

    if (!rotateCCW(SEARCH_TURN_DEG))
      return stopTagWithError(-3);

    searchTurns++;

    delay(100);

    tag = rangeAndHeadingToTag(tagID);
  }

  if (tag.status == -2)
    return stopTagWithError(-2);

  if (tag.status != 1)
    return stopTagWithError(-1);

  Serial.print(F("FOUND | Tag "));
  Serial.println(tagID);


  // Initial alignment before driving.
  int aligned = alignTagToCenter(tagID, tag);

  if (aligned != 1)
    return aligned;


  // Continuous forward movement.
  // There is no 40 cm stop-and-recheck.
  return liveDriveToTag(tagID, tag);
}
