
/*
  TME 441 - AC-Team 9
  driving.ino

  Encoder-controlled robot movements.

  Uses:
  - Dr. Ford's PID motor controller
  - Acceleration and deceleration profiles
  - Wheel encoders to measure distance
  - Dead reckoning to track X, Y, and heading

  Positive distance = forward
  Negative distance = backward
  Positive rotation = counterclockwise
*/

#include <math.h>

// ====================================
// ROBOT PHYSICAL CONSTANTS
// ====================================

// Wheel diameter in centimeters
const float WHEEL_DIAMETER_CM = 8.0f;

// Distance between wheel centers
const float TRACK_WIDTH_CM = 17.85f;

// Encoder ticks per wheel revolution
const float ENCODER_TICKS_PER_WHEEL_REV = 1632.67f;


// ====================================
// MOTOR SPEED SETTINGS
// ====================================

// Maximum requested wheel speed (ticks/sec)
const long MAX_SPEED_TICKS = 1100L;

// Minimum requested wheel speed
const long MIN_SPEED_TICKS = 180L;

// Acceleration increase per control cycle
const long ACCEL_STEP_TICKS = 30L;

// Start slowing down near destination
const long RAMP_DOWN_TICKS = 1500L;


// ====================================
// SAFETY SETTINGS
// ====================================

// Maximum time for a movement
const unsigned long MOVEMENT_TIMEOUT_MS = 60000UL;

// Stop if either wheel makes no progress
const unsigned long NO_PROGRESS_TIMEOUT_MS = 3000UL;

// Wait briefly after stopping
const unsigned long BRAKE_SETTLE_MS = 200UL;


// ====================================
// STOP DRIVING
// ====================================

void stopDriving() {

  // Record any remaining encoder movement
  updateOdometry();

  // Stop both motors
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  // Reset Dr. Ford's PID controller
  resetPID();
}


// ====================================
// MOVE BOTH WHEELS
// ====================================

// Each wheel receives its own signed distance.
// This function handles forward, reverse,
// and in-place rotations.

bool moveWheels(float distance_l_cm,
                float distance_r_cm) {

  // Stop before starting a new movement
  stopDriving();

  // Check physical calibration constants
  if (WHEEL_DIAMETER_CM <= 0.0f ||
      TRACK_WIDTH_CM <= 0.0f ||
      ENCODER_TICKS_PER_WHEEL_REV <= 0.0f) {

    return false;
  }

  // ====================================
  // CONVERT DISTANCE TO ENCODER TICKS
  // ====================================

  const long ticks_per_cm_scaled =
      (long)(ENCODER_TICKS_PER_WHEEL_REV * 100.0f /
      (PI * WHEEL_DIAMETER_CM));

  // fabsf preserves fractional centimeter distances
  const long target_l =
      (fabsf(distance_l_cm) *
       ticks_per_cm_scaled) / 100L;

  const long target_r =
      (fabsf(distance_r_cm) *
       ticks_per_cm_scaled) / 100L;

  // Determine wheel directions
  const int dir_l =
      (distance_l_cm < 0.0f) ? -1 : 1;

  const int dir_r =
      (distance_r_cm < 0.0f) ? -1 : 1;

  // Record starting encoder positions
  const long start_l = enc_l.read();
  const long start_r = enc_r.read();

  // Reset PID before movement
  resetPID();

  // Track completion of each wheel
  bool finished_l = (target_l == 0);
  bool finished_r = (target_r == 0);

  // Timing variables
  const unsigned long start_time = millis();

  unsigned long last_prog_l = start_time;
  unsigned long last_prog_r = start_time;

  long max_p_l = 0;
  long max_p_r = 0;

  // Independent acceleration profiles
  long accel_speed_l = MIN_SPEED_TICKS;
  long accel_speed_r = MIN_SPEED_TICKS;


  // ====================================
  // CONTINUE UNTIL BOTH WHEELS FINISH
  // ====================================

  while (!finished_l || !finished_r) {

    // EXTRA LABOR:
    // Continuously update X, Y, and heading
    // from wheel encoder measurements.
    updateOdometry();

    unsigned long now = millis();

    // Find encoder progress for each wheel
    long prog_l =
        dir_l * (enc_l.read() - start_l);

    long prog_r =
        dir_r * (enc_r.read() - start_r);

    // Update last recorded forward progress
    if (prog_l > max_p_l) {

      max_p_l = prog_l;
      last_prog_l = now;
    }

    if (prog_r > max_p_r) {

      max_p_r = prog_r;
      last_prog_r = now;
    }

    // Check whether each wheel reached target
    if (prog_l >= target_l) {
      finished_l = true;
    }

    if (prog_r >= target_r) {
      finished_r = true;
    }

    // Stop when both wheels finish
    if (finished_l && finished_r) {
      break;
    }


    // ====================================
    // MOVEMENT SAFETY CHECKS
    // ====================================

    // Stop on timeout or encoder stall
    if ((now - start_time >= MOVEMENT_TIMEOUT_MS) ||

        (!finished_l &&
         now - last_prog_l >= NO_PROGRESS_TIMEOUT_MS) ||

        (!finished_r &&
         now - last_prog_r >= NO_PROGRESS_TIMEOUT_MS)) {

      Serial.println("ERROR: Movement timeout or encoder stall");

      stopDriving();
      return false;
    }


    // Requested wheel speeds
    long speed_l = 0;
    long speed_r = 0;


    // ====================================
    // LEFT WHEEL SPEED PROFILE
    // ====================================

    if (!finished_l) {

      // Remaining encoder ticks
      long rem_l = target_l - prog_l;

      // Gradually accelerate
      if (accel_speed_l < MAX_SPEED_TICKS) {

        accel_speed_l += ACCEL_STEP_TICKS;

        if (accel_speed_l > MAX_SPEED_TICKS) {
          accel_speed_l = MAX_SPEED_TICKS;
        }
      }

      // Calculate deceleration speed limit
      long decel_speed_l = MAX_SPEED_TICKS;

      if (rem_l < RAMP_DOWN_TICKS &&
          RAMP_DOWN_TICKS > 0) {

        decel_speed_l =
            MIN_SPEED_TICKS +
            ((MAX_SPEED_TICKS - MIN_SPEED_TICKS)
             * rem_l) / RAMP_DOWN_TICKS;
      }

      // Use the lower of acceleration
      // and deceleration speed limits
      long target_speed_l =
          min(accel_speed_l, decel_speed_l);

      if (target_speed_l < MIN_SPEED_TICKS &&
          rem_l > 0) {

        target_speed_l = MIN_SPEED_TICKS;
      }

      speed_l = dir_l * target_speed_l;
    }


    // ====================================
    // RIGHT WHEEL SPEED PROFILE
    // ====================================

    if (!finished_r) {

      // Remaining encoder ticks
      long rem_r = target_r - prog_r;

      // Gradually accelerate
      if (accel_speed_r < MAX_SPEED_TICKS) {

        accel_speed_r += ACCEL_STEP_TICKS;

        if (accel_speed_r > MAX_SPEED_TICKS) {
          accel_speed_r = MAX_SPEED_TICKS;
        }
      }

      // Calculate deceleration speed limit
      long decel_speed_r = MAX_SPEED_TICKS;

      if (rem_r < RAMP_DOWN_TICKS &&
          RAMP_DOWN_TICKS > 0) {

        decel_speed_r =
            MIN_SPEED_TICKS +
            ((MAX_SPEED_TICKS - MIN_SPEED_TICKS)
             * rem_r) / RAMP_DOWN_TICKS;
      }

      // Choose the lower speed limit
      long target_speed_r =
          min(accel_speed_r, decel_speed_r);

      if (target_speed_r < MIN_SPEED_TICKS &&
          rem_r > 0) {

        target_speed_r = MIN_SPEED_TICKS;
      }

      speed_r = dir_r * target_speed_r;
    }


    // ====================================
    // APPLY PID MOTOR CONTROL
    // ====================================

    // Dr. Ford's existing PID function
    // controls the actual motor power.
    updatePID(speed_l, speed_r, false);

    // Halt individual wheels when finished
    if (finished_l) {
      motorHalt(LMOTOR);
    }

    if (finished_r) {
      motorHalt(RMOTOR);
    }

    // Run control loop approximately every 20 ms
    delay(20);
  }


  // ====================================
  // FINISH MOVEMENT
  // ====================================

  stopDriving();

  // Allow robot to settle
  delay(BRAKE_SETTLE_MS);

  resetPID();

  return true;
}


// ====================================
// DRIVE STRAIGHT
// ====================================

bool driveDistance(float distance_cm) {

  // Equal wheel distances = straight travel
  return moveWheels(distance_cm, distance_cm);
}


// ====================================
// ROTATE ROBOT
// ====================================

bool rotateCCW(float angle_degrees) {

  // Calculate distance each wheel must travel
  // for an in-place rotation.

  const float wheel_arc_cm =
      (PI * TRACK_WIDTH_CM * angle_degrees) /
      360.0f;

  // Left wheel backward, right wheel forward
  // produces a counterclockwise turn.
  return moveWheels(-wheel_arc_cm, wheel_arc_cm);
}
