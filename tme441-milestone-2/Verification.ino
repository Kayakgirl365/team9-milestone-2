
/*
  TME 441 - Milestone 2
  AC-Team 9

  HUSKYLENS CAMERA VERIFICATION TEST

  Robot does NOT move.

  Prints:
  - Tag ID
  - Tag height in pixels
  - Horizontal position in pixels
  - Estimated distance in cm
  - Estimated heading in degrees
  - Distance measurement error
  - Heading measurement error
*/

#include <HUSKYLENS.h>
#include <math.h>

HUSKYLENS cam;


// =====================================
// CAMERA CALIBRATION
// =====================================

// Change this to test Tag 1, 2, or 3
const int TAG_ID = 1;

// Values from our camera calibration
const float RANGE_SCALE = 3360.0f;
const float RANGE_OFFSET_CM = 0.0f;

const float CAMERA_CENTER_X = 165.5f;
const float CAMERA_FOCAL_PIXELS = 300.4f;

// Measured distance from lens to robot front.
// Leave -1 if not measured yet.
const float CAMERA_TO_FRONT_CM = -1.0f;


// =====================================
// ACTUAL MEASUREMENTS
// =====================================

// Enter the REAL distance measured with a ruler.
// Use -1 to disable distance error calculation.
const float ACTUAL_DISTANCE_CM = -1.0f;

// Enter the REAL angle measured with a protractor.
// Positive = tag left; negative = tag right.
// Use 999 to disable angle error calculation.
const float ACTUAL_ANGLE_DEG = 999.0f;


// =====================================
// SETUP
// =====================================

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);

  delay(1000);

  if (!cam.begin(Serial3)) {
    Serial.println("HuskyLens connection failed!");
    while (true) {
      delay(1000);
    }
  }

  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  delay(500);

  Serial.println("CAMERA VERIFICATION START");

  // Column labels for copying into Excel
  Serial.println(
    "time_ms,tag_id,status,height_px,x_px,"
    "lens_cm,front_cm,heading_deg,"
    "distance_error_cm,heading_error_deg"
  );
}


// =====================================
// LOOP - READ CAMERA EVERY 500 MS
// =====================================

void loop() {

  bool received = false;

  // Retry camera communication if necessary
  for (int attempt = 0; attempt < 3; attempt++) {
    if (cam.requestBlocks(TAG_ID)) {
      received = true;
      break;
    }
    delay(35);
  }

  // Communication failure
  if (!received) {
    printMissing("CAMERA_ERROR");
    delay(500);
    return;
  }

  bool found = false;

  while (cam.available()) {

    HUSKYLENSResult tag = cam.read();

    // Only use the requested AprilTag
    if (tag.command != COMMAND_RETURN_BLOCK ||
        tag.ID != TAG_ID ||
        tag.height <= 0) {
      continue;
    }

    found = true;

    // Calculate distance from tag pixel height
    float lensDistance =
        RANGE_SCALE / tag.height - RANGE_OFFSET_CM;

    // Calculate heading from horizontal position
    float heading =
        atan2f(
          CAMERA_CENTER_X - tag.xCenter,
          CAMERA_FOCAL_PIXELS
        ) * 180.0f / PI;

    // Print timestamp and tag information
    Serial.print(millis());
    Serial.print(",");
    Serial.print(tag.ID);
    Serial.print(",DETECTED,");
    Serial.print(tag.height);
    Serial.print(",");
    Serial.print(tag.xCenter);
    Serial.print(",");
    Serial.print(lensDistance, 2);
    Serial.print(",");

    // Estimated distance from robot's front
    if (CAMERA_TO_FRONT_CM >= 0.0f) {
      Serial.print(
        lensDistance - CAMERA_TO_FRONT_CM, 2
      );
    } else {
      Serial.print("NA");
    }

    Serial.print(",");
    Serial.print(heading, 2);
    Serial.print(",");

    // Distance error if actual distance is known
    if (ACTUAL_DISTANCE_CM >= 0.0f) {
      float distanceError =
          fabsf(lensDistance - ACTUAL_DISTANCE_CM);

      Serial.print(distanceError, 2);
    } else {
      Serial.print("NA");
    }

    Serial.print(",");

    // Heading error if actual angle is known
    if (ACTUAL_ANGLE_DEG != 999.0f) {
      float headingError =
          fabsf(heading - ACTUAL_ANGLE_DEG);

      Serial.println(headingError, 2);
    } else {
      Serial.println("NA");
    }
  }

  // No matching tag detected
  if (!found) {
    printMissing("NOT_SEEN");
  }

  delay(500);
}


// =====================================
// PRINT FAILURE OR MISSING TAG
// =====================================

void printMissing(const char* status) {

  Serial.print(millis());
  Serial.print(",");
  Serial.print(TAG_ID);
  Serial.print(",");
  Serial.print(status);
  Serial.println(",NA,NA,NA,NA,NA,NA,NA");
}
