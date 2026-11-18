
/*
  TME 441 - Milestone 2
  AC-Team 9

  camera.ino

  HuskyLens AprilTag detection
  with Serial Monitor debugging.
*/

// ====================================
// DEBUG SETTINGS
// ====================================

const bool CAMERA_DEBUG = true;


// ====================================
// CAMERA CALIBRATION
// ====================================

// These are the current GitHub values.
// Keep your LOCAL calibrated values
// if you have already improved them.

const float CAMERA_CENTER_X = 160.0f;
const float CAMERA_FOCAL_PIXELS = 320.0f;

const float CAMERA_YAW_OFFSET_DEG = 0.0f;

const float RANGE_SCALE = 3489.0f;
const float RANGE_OFFSET_CM = 3.0f;

const float CAMERA_TO_FRONT_CM = 0.0f;


// ====================================
// READ APRILTAG
// ====================================

TagPose rangeAndHeadingToTag(int tagID) {

  TagPose pose = {0, -1.0f, 0.0f, 0, 0};

  bool received = false;

  // Retry communication up to three times
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
  // PROCESS CAMERA BLOCKS
  // ====================================

  while (cam.available()) {

    HUSKYLENSResult block = cam.read();

    // Ignore incorrect tag IDs
    if (block.command != COMMAND_RETURN_BLOCK ||
        block.ID != tagID ||
        block.height <= 0) {

      continue;
    }

    pose.status = 1;

    pose.xCenter = block.xCenter;
    pose.height = block.height;


    // ====================================
    // CALCULATE DISTANCE
    // ====================================

    pose.range =
        RANGE_SCALE / block.height -
        RANGE_OFFSET_CM;


    // ====================================
    // CALCULATE HEADING
    // ====================================

    // Positive heading = tag to the left
    // Negative heading = tag to the right

    pose.heading =
        atan2f(
          CAMERA_CENTER_X - block.xCenter,
          CAMERA_FOCAL_PIXELS
        ) * 180.0f / PI;

    pose.heading += CAMERA_YAW_OFFSET_DEG;


    // ====================================
    // PRINT CAMERA DATA
    // ====================================

    if (CAMERA_DEBUG) {

      Serial.print("CAMERA | t_ms=");
      Serial.print(millis());

      Serial.print(" | id=");
      Serial.print(block.ID);

      Serial.print(" | x_px=");
      Serial.print(block.xCenter);

      Serial.print(" | y_px=");
      Serial.print(block.yCenter);

      Serial.print(" | width_px=");
      Serial.print(block.width);

      Serial.print(" | height_px=");
      Serial.print(block.height);

      Serial.print(" | x_error_px=");
      Serial.print(
        block.xCenter - CAMERA_CENTER_X, 1
      );

      Serial.print(" | lens_cm=");
      Serial.print(pose.range, 2);

      Serial.print(" | front_cm=");
      Serial.print(
        pose.range - CAMERA_TO_FRONT_CM, 2
      );

      Serial.print(" | heading_deg=");
      Serial.println(pose.heading, 2);
    }

    return pose;
  }


  // ====================================
  // TAG NOT DETECTED
  // ====================================

  if (CAMERA_DEBUG) {

    Serial.print("CAMERA | Tag ");
    Serial.print(tagID);
    Serial.println(" NOT VISIBLE");
  }

  return pose;
}


// ====================================
// DISTANCE FROM ROBOT FRONT
// ====================================

float distanceFromFront(TagPose tag) {

  return tag.range - CAMERA_TO_FRONT_CM;
}

