
/*
  TME 441 - Milestone 2
  AC-Team 9

  camera.ino

  HuskyLens AprilTag detection with Serial debugging.

  Measures:
  - Tag ID
  - Tag X and Y image coordinates
  - Tag width and height in pixels
  - Distance from camera lens
  - Estimated front clearance
  - Heading toward tag

  No changes to navigation behavior.
*/

// ====================================
// DEBUG SETTINGS
// ====================================

// Set false when finished collecting data.
const bool CAMERA_DEBUG = true;


// ====================================
// CAMERA CALIBRATION
// ====================================

// Use the calibration values from the
// working version of your robot.

// Horizontal center of HuskyLens image
const float CAMERA_CENTER_X = 160.0f;

// Focal length in pixels
const float CAMERA_FOCAL_PIXELS = 320.0f;

// Camera mounting angle correction
const float CAMERA_YAW_OFFSET_DEG = 0.0f;

// Distance calibration based on tag height
const float RANGE_SCALE = 3489.0f;
const float RANGE_OFFSET_CM = 3.0f;

// Camera lens to robot front, in cm.
// Replace with your measured offset.
const float CAMERA_TO_FRONT_CM = 0.0f;


// ====================================
// MEASURE TAG RANGE AND HEADING
// ====================================

TagPose rangeAndHeadingToTag(int tagID) {

  // Default: tag not detected.
  TagPose pose = {0, -1.0f, 0.0f, 0, 0};

  bool received = false;

  // Request only the specified tag ID.
  // Retry communication up to three times.
  for (int attempt = 0; attempt < 3; attempt++) {

    if (cam.requestBlocks(tagID)) {
      received = true;
      break;
    }

    delay(35);
  }

  // ====================================
  // CAMERA COMMUNICATION ERROR
  // ====================================

  if (!received) {

    pose.status = -2;

    if (CAMERA_DEBUG) {
      Serial.print("CAMERA ERROR | Tag ");
      Serial.println(tagID);
    }

    return pose;
  }


  // ====================================
  // READ DETECTED TAG
  // ====================================

  while (cam.available()) {

    HUSKYLENSResult block = cam.read();

    // Ignore incorrect IDs or invalid readings.
    if (block.command != COMMAND_RETURN_BLOCK ||
        block.ID != tagID ||
        block.height <= 0) {

      continue;
    }

    // Correct tag found.
    pose.status = 1;

    // Store pixel measurements.
    pose.xCenter = block.xCenter;
    pose.height = block.height;

    // Calculate distance to tag.
    pose.range =
        RANGE_SCALE / block.height - RANGE_OFFSET_CM;

    // Calculate heading toward tag.
    // Positive = left
    // Negative = right
    pose.heading =
        atan2f(
          CAMERA_CENTER_X - block.xCenter,
          CAMERA_FOCAL_PIXELS
        ) * 180.0f / PI;

    // Correct for camera mounting angle.
    pose.heading += CAMERA_YAW_OFFSET_DEG;


    // ====================================
    // CAMERA DEBUG OUTPUT
    // ====================================

    if (CAMERA_DEBUG) {

      Serial.println("========== CAMERA DATA ==========");

      Serial.print("Time (ms): ");
      Serial.println(millis());

      Serial.print("Detected Tag ID: ");
      Serial.println(block.ID);

      Serial.print("X Center (px): ");
      Serial.println(block.xCenter);

      Serial.print("Y Center (px): ");
      Serial.println(block.yCenter);

      Serial.print("Width (px): ");
      Serial.println(block.width);

      Serial.print("Height (px): ");
      Serial.println(block.height);

      Serial.print("X Error (px): ");
      Serial.println(
        block.xCenter - CAMERA_CENTER_X, 1
      );

      Serial.print("Distance from Lens (cm): ");
      Serial.println(pose.range, 2);

      Serial.print("Distance from Front (cm): ");
      Serial.println(
        pose.range - CAMERA_TO_FRONT_CM, 2
      );

      Serial.print("Heading (degrees): ");
      Serial.println(pose.heading, 2);

      Serial.println("=================================");
    }

    return pose;
  }


  // ====================================
  // TAG NOT VISIBLE
  // ====================================

  if (CAMERA_DEBUG) {

    Serial.print("CAMERA | Looking for Tag ");
    Serial.print(tagID);
    Serial.println(" | NOT VISIBLE");
  }

  return pose;
}


// ====================================
// DISTANCE FROM ROBOT FRONT
// ====================================

float distanceFromFront(TagPose tag) {

  return tag.range - CAMERA_TO_FRONT_CM;
}

