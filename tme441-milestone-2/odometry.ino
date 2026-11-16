/*
  odometry.ino - Extra labor: DEAD RECKONING

  The starting point is (X, Y) = (0, 0).
  The starting direction is theta = 0 degrees.
  +X means forward from the starting pose.
  +Y means left from the starting pose.
  Positive theta means counterclockwise (left).

  This uses ENCODER readings, not movement timers.
*/

// Store the estimated position of the CENTER of the robot.
struct RobotPose {
  float x_cm;
  float y_cm;
  float theta_rad;  // Angle stored in radians
};

RobotPose robotPose = {0.0f, 0.0f, 0.0f};

// Remember the last encoder values to measure each small movement.
long previousLeftTicks = 0;
long previousRightTicks = 0;
bool odometryStarted = false;

// Convert any angle to the range -180 to +180 degrees (in radians).
float wrapAngle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

// Call ONCE at the start of the mission, before the wheels move.
void resetOdometry() {
  robotPose.x_cm = 0.0f;
  robotPose.y_cm = 0.0f;
  robotPose.theta_rad = 0.0f;
  previousLeftTicks = enc_l.read();
  previousRightTicks = enc_r.read();
  odometryStarted = true;
}

// Called repeatedly while driving or turning.
// Calculates the change in X, Y, and heading from both encoders.
void updateOdometry() {
  if (!odometryStarted) return;

  // Read both encoders and find movement since the last reading.
  long leftTicks = enc_l.read();
  long rightTicks = enc_r.read();
  long deltaLeftTicks = leftTicks - previousLeftTicks;
  long deltaRightTicks = rightTicks - previousRightTicks;

  previousLeftTicks = leftTicks;
  previousRightTicks = rightTicks;

  if (deltaLeftTicks == 0 && deltaRightTicks == 0) return;

  // Convert encoder ticks into distance for each wheel (cm).
  float cmPerTick = (PI * WHEEL_DIAMETER_CM) /
                    ENCODER_TICKS_PER_WHEEL_REV;
  float leftCm = deltaLeftTicks * cmPerTick;
  float rightCm = deltaRightTicks * cmPerTick;

  // Both wheels moving together changes position.
  // One wheel going backward and the other forward changes heading.
  float distanceCm = (leftCm + rightCm) / 2.0f;
  float deltaTheta = (rightCm - leftCm) / TRACK_WIDTH_CM;

  // Use the halfway heading for better tracking during curved motion.
  float middleTheta = robotPose.theta_rad + deltaTheta / 2.0f;
  robotPose.x_cm += distanceCm * cosf(middleTheta);
  robotPose.y_cm += distanceCm * sinf(middleTheta);
  robotPose.theta_rad = wrapAngle(robotPose.theta_rad + deltaTheta);
}

// Print the estimated position after each movement or waypoint.
void printPose() {
  updateOdometry();
  Serial.print(F("X_cm:")); Serial.print(robotPose.x_cm, 1);
  Serial.print(F(" Y_cm:")); Serial.print(robotPose.y_cm, 1);
  Serial.print(F(" theta_deg:"));
  Serial.println(robotPose.theta_rad * 180.0f / PI, 1);
}

// Called AFTER reaching Tag 3 (or during a short odometry test).
// Turn toward the starting position, drive there, then face the
// same direction as when the robot started (theta = 0 degrees).
bool returnHome() {
  const float HOME_TOLERANCE_CM = 5.0f;
  const float ANGLE_TOLERANCE_DEG = 5.0f;

  // Up to two passes correct for small encoder rounding errors.
  for (int attempt = 0; attempt < 2; attempt++) {
    updateOdometry();

    // Find the vector from current position to the starting point.
    float homeX = -robotPose.x_cm;
    float homeY = -robotPose.y_cm;
    float homeDistance = sqrtf(homeX * homeX + homeY * homeY);

    if (homeDistance <= HOME_TOLERANCE_CM) break;

    // Reject an unreasonable position estimate.
    if (homeDistance > 300.0f) {
      Serial.println(F("ERROR: Home estimate is outside allowed range"));
      return false;
    }

    // Calculate the angle needed to face the starting point.
    float targetTheta = atan2f(homeY, homeX);
    float turnRadians = wrapAngle(targetTheta - robotPose.theta_rad);
    float turnDegrees = turnRadians * 180.0f / PI;

    Serial.print(F("Return turn (deg): "));
    Serial.println(turnDegrees, 1);
    if (!rotateCCW(turnDegrees)) return false;

    Serial.print(F("Return drive (cm): "));
    Serial.println(homeDistance, 1);
    if (!driveDistance(homeDistance)) return false;
  }

  // The robot may be home but facing a different direction.
  // Turn back to the original heading of zero degrees.
  updateOdometry();
  float headingCorrection = wrapAngle(-robotPose.theta_rad) * 180.0f / PI;
  if (fabsf(headingCorrection) > 1.0f) {
    if (!rotateCCW(headingCorrection)) return false;
  }

  stopDriving();
  updateOdometry();
  printPose();

  // Check the estimated final error; this is NOT a ruler measurement.
  float positionError = sqrtf(robotPose.x_cm * robotPose.x_cm +
                               robotPose.y_cm * robotPose.y_cm);
  float headingError = fabsf(robotPose.theta_rad * 180.0f / PI);

  Serial.print(F("Estimated position error (cm): "));
  Serial.println(positionError, 1);
  Serial.print(F("Estimated heading error (deg): "));
  Serial.println(headingError, 1);

  return (positionError <= HOME_TOLERANCE_CM &&
          headingError <= ANGLE_TOLERANCE_DEG);
}
