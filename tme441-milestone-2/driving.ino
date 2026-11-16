/* Driving and Trajectory Utilities
 * Implements non-conflicting acceleration and deceleration profiles:
 * - Time-based smooth ramping for acceleration (using small step ticks).
 * - Distance-based ramping for deceleration to ensure zero-overshoot stopping.
 * Optimized with integer arithmetic and debugging output enabled.
 */

// Robot Physical Calibration Constants
const float WHEEL_DIAMETER_CM = 8.0f;           
const float TRACK_WIDTH_CM = 17.85f;              
const float ENCODER_TICKS_PER_WHEEL_REV = 1632.67f;  

// Profile Speed & Slew Settings (in encoder ticks/sec)
const long MAX_SPEED_TICKS = 1100L;  
const long MIN_SPEED_TICKS = 180L;   
const long ACCEL_STEP_TICKS = 30L;    // Small step for smooth, visible acceleration
const long RAMP_DOWN_TICKS = 1500L;  // Distance threshold over which to ramp down (~23 cm)

const unsigned long MOVEMENT_TIMEOUT_MS = 60000UL;
const unsigned long NO_PROGRESS_TIMEOUT_MS = 3000UL;
const unsigned long BRAKE_SETTLE_MS = 200UL;

void stopDriving() {
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
  resetPID();
}

/* Move each wheel through a signed distance with non-conflicting accel/decel profiling */
bool moveWheels(float distance_l_cm, float distance_r_cm) {
  stopDriving();

  if (WHEEL_DIAMETER_CM <= 0.0f || TRACK_WIDTH_CM <= 0.0f || ENCODER_TICKS_PER_WHEEL_REV <= 0.0f) {
    return false;
  }

  // Pre-scale ticks per cm factor to integer arithmetic
  const long ticks_per_cm_scaled = (long)(ENCODER_TICKS_PER_WHEEL_REV * 100.0f / (3.14159265f * WHEEL_DIAMETER_CM));
  
  const long target_l = (abs(distance_l_cm) * ticks_per_cm_scaled) / 100L;
  const long target_r = (abs(distance_r_cm) * ticks_per_cm_scaled) / 100L;
  const int dir_l = (distance_l_cm < 0.0f) ? -1 : 1;
  const int dir_r = (distance_r_cm < 0.0f) ? -1 : 1;

  const long start_l = enc_l.read();
  const long start_r = enc_r.read();
  resetPID();

  bool finished_l = (target_l == 0);
  bool finished_r = (target_r == 0);
  const unsigned long start_time = millis();
  unsigned long last_prog_l = start_time;
  unsigned long last_prog_r = start_time;
  long max_p_l = 0;
  long max_p_r = 0;

  // Independent acceleration tracking variables for each wheel
  long accel_speed_l = MIN_SPEED_TICKS;
  long accel_speed_r = MIN_SPEED_TICKS;

  while (!finished_l || !finished_r) {
    unsigned long now = millis();
    long prog_l = dir_l * (enc_l.read() - start_l);
    long prog_r = dir_r * (enc_r.read() - start_r);

    if (prog_l > max_p_l) { max_p_l = prog_l; last_prog_l = now; }
    if (prog_r > max_p_r) { max_p_r = prog_r; last_prog_r = now; }

    if (prog_l >= target_l) finished_l = true;
    if (prog_r >= target_r) finished_r = true;
    if (finished_l && finished_r) break;

    if ((now - start_time >= MOVEMENT_TIMEOUT_MS) ||
        (!finished_l && now - last_prog_l >= NO_PROGRESS_TIMEOUT_MS) ||
        (!finished_r && now - last_prog_r >= NO_PROGRESS_TIMEOUT_MS)) {
      stopDriving();
      return false;
    }

    long speed_l = 0;
    long speed_r = 0;

    // --- Left Wheel Profile ---
    if (!finished_l) {
      long rem_l = target_l - prog_l;
      
      // 1. Acceleration ramp ceiling
      if (accel_speed_l < MAX_SPEED_TICKS) {
        accel_speed_l += ACCEL_STEP_TICKS;
        if (accel_speed_l > MAX_SPEED_TICKS) accel_speed_l = MAX_SPEED_TICKS;
      }
      
      // 2. Deceleration limit based on remaining distance
      long decel_speed_l = MAX_SPEED_TICKS;
      if (rem_l < RAMP_DOWN_TICKS && RAMP_DOWN_TICKS > 0) {
        decel_speed_l = MIN_SPEED_TICKS + ((MAX_SPEED_TICKS - MIN_SPEED_TICKS) * rem_l) / RAMP_DOWN_TICKS;
      }

      // 3. Take the minimum so acceleration and deceleration never fight
      long target_speed_l = min(accel_speed_l, decel_speed_l);
      if (target_speed_l < MIN_SPEED_TICKS && rem_l > 0) target_speed_l = MIN_SPEED_TICKS;

      speed_l = dir_l * target_speed_l;
    }

    // --- Right Wheel Profile ---
    if (!finished_r) {
      long rem_r = target_r - prog_r;
      
      // 1. Acceleration ramp ceiling
      if (accel_speed_r < MAX_SPEED_TICKS) {
        accel_speed_r += ACCEL_STEP_TICKS;
        if (accel_speed_r > MAX_SPEED_TICKS) accel_speed_r = MAX_SPEED_TICKS;
      }
      
      // 2. Deceleration limit based on remaining distance
      long decel_speed_r = MAX_SPEED_TICKS;
      if (rem_r < RAMP_DOWN_TICKS && RAMP_DOWN_TICKS > 0) {
        decel_speed_r = MIN_SPEED_TICKS + ((MAX_SPEED_TICKS - MIN_SPEED_TICKS) * rem_r) / RAMP_DOWN_TICKS;
      }

      // 3. Take the minimum
      long target_speed_r = min(accel_speed_r, decel_speed_r);
      if (target_speed_r < MIN_SPEED_TICKS && rem_r > 0) target_speed_r = MIN_SPEED_TICKS;

      speed_r = dir_r * target_speed_r;
    }

    // Report enabled (true) to view curves on Serial Plotter
    updatePID(speed_l, speed_r, true);

    if (finished_l) motorHalt(LMOTOR);
    if (finished_r) motorHalt(RMOTOR);
    delay(20);  // Synchronized with PID_INT (20ms update interval)
  }

  stopDriving();
  delay(BRAKE_SETTLE_MS);  
  resetPID();
  return true;
}

/* Drive straight with smooth slew profiling */
bool driveDistance(float distance_cm) {
  return moveWheels(distance_cm, distance_cm);
}

/* Rotate in place with smooth slew profiling */
bool rotateCCW(float angle_degrees) {
  const float wheel_arc_cm = (3.14159265f * TRACK_WIDTH_CM * angle_degrees) / 360.0f;
  return moveWheels(-wheel_arc_cm, wheel_arc_cm);
}
