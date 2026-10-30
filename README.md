# TME 441 Milestone 2

Arduino sketch for navigating a robot between AprilTags using a HuskyLens camera.
The program searches for tags 1 through 3, approaches each tag, and repeats.

This solution works reasonably well, as long as the tags are well-lit and close
enough to the robot. But under non-ideal conditions, the robot periodically
loses sight of the tag, or overshoots the angle and misses the tag entirely.

Your task is to test this solution, discover its limitations, and then redesign
it to be more robust and reliable.

## Files

- `tme441-milestone-2.ino`: sketch entry point and HuskyLens setup.
- `camera.ino`: camera queries and tag distance estimation.
- `navigation.ino`: tag search, alignment, and approach logic.

## Requirements

- An Arduino board with `Serial3` (such as an Arduino Mega).
- A HuskyLens camera connected over serial and configured for **Tag Recognition** with the serial protocol set to **9600**.
- The Arduino `Encoder`, `Servo`, and `HuskyLens` libraries.
- Your own `driving.ino` and `motor_control.h` files. They are not included in this repository; the sketch expects motor functions such as `initMotors()`, `driveDistance()`, and `rotateCCW()`.

Open the sketch in the Arduino IDE, add the required libraries and motor-control files.
