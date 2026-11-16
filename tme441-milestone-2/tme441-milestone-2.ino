
/*
  TME 441 - AC-Team 9
  Milestone 2 - Motor Debug Test

  Test both motors individually.
  Check whether the encoders detect movement.

  Left motor: 2 seconds
  Right motor: 2 seconds

  Keep both wheels off the ground during testing.
*/

#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include <math.h>

// Keep the camera object for the other Arduino tabs.
HUSKYLENS cam;

// Keep this struct for camera.ino and navigation.ino.
struct TagPose {
  int status;
  float range;
  float heading;
  int xCenter;
  int height;
};


void setup() {

  // Start Serial Monitor.
  Serial.begin(9600);
  delay(1000);

  // Initialize motor pins.
  initMotors();

  // Make sure both motors start stopped.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  Serial.println("===== MOTOR TEST START =====");


  // =====================================
  // TEST 1: LEFT MOTOR
  // =====================================

  Serial.println("Testing LEFT motor");

  // Save encoder position before moving.
  long leftStart = enc_l.read();

  // Run left motor at PWM 180.
  motorOn(LMOTOR, 180);

  // Run for 2 seconds.
  delay(2000);

  // Stop left motor.
  motorHalt(LMOTOR);

  // Calculate how many encoder ticks changed.
  long leftTicks = enc_l.read() - leftStart;

  Serial.print("LEFT encoder ticks: ");
  Serial.println(leftTicks);

  // Pause before testing right motor.
  delay(1000);


  // =====================================
  // TEST 2: RIGHT MOTOR
  // =====================================

  Serial.println("Testing RIGHT motor");

  // Save encoder position before moving.
  long rightStart = enc_r.read();

  // Run right motor at PWM 180.
  motorOn(RMOTOR, 180);

  // Run for 2 seconds.
  delay(2000);

  // Stop right motor.
  motorHalt(RMOTOR);

  // Calculate encoder movement.
  long rightTicks = enc_r.read() - rightStart;

  Serial.print("RIGHT encoder ticks: ");
  Serial.println(rightTicks);


  // =====================================
  // FINISH TEST
  // =====================================

  // Stop both motors.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  Serial.println("===== MOTOR TEST COMPLETE =====");
}


void loop() {

  // Keep motors stopped after testing.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);
}


