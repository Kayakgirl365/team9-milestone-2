
/*
  TME 441 - Milestone 2
  AC-Team 9

  camera.ino

  Measures:
  - Tag ID
  - Image X and Y
  - Tag height and width
  - Estimated distance
  - Estimated heading
  - Horizontal pixel error
*/

const bool CAMERA_DEBUG = true;


// ====================================
// CAMERA CALIBRATION
// ====================================

const float CAMERA_CENTER_X = 165.5f;
const float CAMERA_FOCAL_PIXELS = 300.4f;

const float CAMERA_YAW_OFFSET_DEG = 0.0f;

const float RANGE_SCALE = 3360.0f;
const float RANGE_OFFSET_CM = 0.0f;

// Measure from lens to robot front.
const float CAMERA_TO_FRONT_CM = 0.0f;


// ====================================
// READ TARGET APRILTAG
// ====================================

TagPose rangeAndHeadingToTag(int tagID) {

  TagPose pose = {0, -1.0f, 0.0f, 0, 0};

  bool received = false;

  // Retry transient camera communication failures.
  for (int attempt = 0; attempt < 3; attempt++) {

    if (cam.requestBlocks(tagID)) {

      received = true;
      break;
    }

    delay(35);
  }


  // ====================================
  // CAMERA ERROR
  // ====================================

  if (!received) {

    pose.status = -2;

    if (CAMERA_DEBUG) {

      Serial.print(F("CAMERA ERROR | requested Tag "));
      Serial.println(tagID);
    }

    return pose;
  }


  // ====================================
  // READ CAMERA BLOCKS
  // ====================================

  while (cam.available()) {

    HUSKYLENSResult block = cam.read();

    if (block.command != COMMAND_RETURN_BLOCK ||
        block.ID != tagID ||
        block.height <= 0) {

      continue;
    }

    pose.status = 1;

    pose.xCenter = block.xCenter;
    pose.height = block.height;


    // ====================================
    // ESTIMATE DISTANCE
    // ====================================

    pose.range =
        RANGE_SCALE / block.height -
        RANGE_OFFSET_CM;


    // ====================================
    // ESTIMATE HEADING
    // ====================================

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

      Serial.print(F("CAMERA | t_ms="));
      Serial.print(millis());

      Serial.print(F(" | id="));
      Serial.print(block.ID);

      Serial.print(F(" | x_px="));
      Serial.print(block.xCenter);

      Serial.print(F(" | y_px="));
      Serial.print(block.yCenter);

      Serial.print(F(" | width_px="));
      Serial.print(block.width);

      Serial.print(F(" | height_px="));
      Serial.print(block.height);

      Serial.print(F(" | x_error_px="));
      Serial.print(
        block.xCenter - CAMERA_CENTER_X, 1
      );

      Serial.print(F(" | lens_cm="));
      Serial.print(pose.range, 2);

      Serial.print(F(" | front_cm="));
      Serial.print(
        pose.range - CAMERA_TO_FRONT_CM, 2
      );

      Serial.print(F(" | heading_deg="));
      Serial.println(pose.heading, 2);
    }

    return pose;
  }


  // ====================================
  // TAG NOT VISIBLE
  // ====================================

  if (CAMERA_DEBUG) {

    Serial.print(F("CAMERA | Tag "));
    Serial.print(tagID);
    Serial.println(F(" NOT VISIBLE"));
  }

  return pose;
}


// ====================================
// DISTANCE FROM ROBOT FRONT
// ====================================

float distanceFromFront(TagPose tag) {

  return tag.range - CAMERA_TO_FRONT_CM;
}

