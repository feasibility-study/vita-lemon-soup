/*
   Trebuchet control sketch for ESP32-S3
   Builds on the 5-motor driver + servo references:
     - Motor 0 (index 0)      -> spool / winch
     - Motors 1-4 (index 1-4) -> drive base (used mainly to SWIVEL the base)
     - Servo on SERVO_PIN     -> holds the arm at full draw, then releases it
*/

#include <Arduino.h>

// =====================================================================
//  Motor / base configuration
// =====================================================================
#define MOTOR_COUNT 5
#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

#define SPOOL_MOTOR       0
#define BASE_MOTOR_START  1
#define BASE_MOTOR_END    4
#define BASE_MOTOR_COUNT  (BASE_MOTOR_END - BASE_MOTOR_START + 1) // 4

const int MOTOR_PINS[MOTOR_COUNT][2] = {
  {1, 2},   // Motor 1 (index 0) -- Spool to move cable
  {3, 4},   // Motor 2 (index 1) -- drive base
  {5, 6},   // Motor 3 (index 2) -- drive base
  {7, 8},   // Motor 4 (index 3) -- drive base
  {9, 44}   // Motor 5 (index 4) -- drive base
            // NOTE: pin 44 here collides with SERVO_PIN below -- re-check
            // your wiring, one of these needs to move to a free pin.
};

/*
  Drive base layout (top view). The 4 base wheels are mounted tangentially
  (not pointing forward/back like a normal car), so the base's job is
  mainly to SWIVEL/spin in place rather than drive in a straight line:

            facing right
    facing up          facing down
            facing left

  Driving every wheel with the "same command" only produces a clean spin
  if each one is wired/oriented consistently for that rotational sense.
  BASE_MOTOR_SIGN lets you flip individual wheels (+1/-1) during
  calibration without touching the swivel logic itself.
*/
const int BASE_MOTOR_SIGN[BASE_MOTOR_COUNT] = { 1, 1, 1, 1 }; // calibrate me

// Compass-style directions the base can be told to face
enum BaseDirection {
  DIR_UP    = 0,
  DIR_RIGHT = 1,
  DIR_DOWN  = 2,
  DIR_LEFT  = 3
};

BaseDirection currentBaseDirection = DIR_UP; // set to your known start orientation

// Open-loop timing calibration -- no encoders/IMU assumed here.
// Time it takes to swivel 90 degrees at SWIVEL_SPEED. Measure and adjust.
#define MS_PER_90_DEG   400
#define SWIVEL_SPEED    180

// =====================================================================
//  Servo (latch) configuration
// =====================================================================
#define SERVO_PIN            44   // conflicts with Motor 5 pin B above, see note
#define SERVO_LATCH_ANGLE    90
#define SERVO_RELEASE_ANGLE  0

// =====================================================================
//  Low-level motor helpers (from your reference, unchanged)
// =====================================================================
void initMotor(int motorIndex) {
  int pinA = MOTOR_PINS[motorIndex][0];
  int pinB = MOTOR_PINS[motorIndex][1];

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  ledcAttach(pinA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinB, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcWrite(pinA, 0);
  ledcWrite(pinB, 0);
}

void setMotor(int motorIndex, int speed) {
  speed = constrain(speed, -255, 255);
  int duty = abs(speed);

  int pinA = MOTOR_PINS[motorIndex][0];
  int pinB = MOTOR_PINS[motorIndex][1];

  if (speed > 0) {
    ledcWrite(pinA, duty);
    ledcWrite(pinB, 0);
  } else if (speed < 0) {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, duty);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, 0);
  }
}

void stopMotor(int motorIndex) {
  setMotor(motorIndex, 0);
}

void stopBase() {
  for (int i = BASE_MOTOR_START; i <= BASE_MOTOR_END; i++) {
    stopMotor(i);
  }
}

// =====================================================================
//  Servo helper (from your reference, unchanged)
// =====================================================================
void servoAngle(int angle) {
  int pulseWidthMicros = map(angle, 0, 180, 500, 2500);
  uint32_t duty = ((uint32_t)pulseWidthMicros * 4096UL) / 20000UL;
  ledcWrite(SERVO_PIN, duty);
}

// =====================================================================
//  DRIVE BASE -- swivel helpers
// =====================================================================

// Spins every base wheel with the same rotational "sense" so the whole
// base rotates in place. spinSpeed > 0 = clockwise, < 0 = counter-clockwise.
void spinBase(int spinSpeed) {
  for (int i = 0; i < BASE_MOTOR_COUNT; i++) {
    int motorIndex = BASE_MOTOR_START + i;
    setMotor(motorIndex, spinSpeed * BASE_MOTOR_SIGN[i]);
  }
}

// Swivels the base until it faces targetDirection.
// Picks whichever way (cw/ccw) is the shorter turn, runs for a time
// proportional to how many 90-degree steps are needed, then stops.
void swivelToDirection(BaseDirection targetDirection, int speed = SWIVEL_SPEED) {
  int diffCW = ((int)targetDirection - (int)currentBaseDirection + 4) % 4; // 0..3

  if (diffCW == 0) {
    return; // already facing that way
  }

  int stepsCW  = diffCW;
  int stepsCCW = 4 - diffCW;

  int rotationSteps;
  int spinSpeed;

  if (stepsCW <= stepsCCW) {
    rotationSteps = stepsCW;
    spinSpeed = speed;     // clockwise
  } else {
    rotationSteps = stepsCCW;
    spinSpeed = -speed;    // counter-clockwise
  }

  unsigned long duration = (unsigned long)rotationSteps * MS_PER_90_DEG;

  spinBase(spinSpeed);
  delay(duration);
  stopBase();

  currentBaseDirection = targetDirection;
}

// =====================================================================
//  SPOOL -- winch + servo latch helpers
// =====================================================================

// direction > 0 winds in (pulls the arm back), direction < 0 unwinds.
// Runs at `speed` for durationMs, then stops. Swap the sign convention
// below if your wiring winds in the opposite direction.
void moveSpool(int direction, unsigned long durationMs, int speed = 200) {
  int spoolSpeed = (direction >= 0) ? speed : -speed;
  setMotor(SPOOL_MOTOR, spoolSpeed);
  delay(durationMs);
  stopMotor(SPOOL_MOTOR);
}

// Full loading sequence:
//  1. wind the spool to pull the arm back (draw it down)
//  2. servo swings to 90 degrees to latch/hold the arm at full draw
//  3. spool unwinds so the cable goes slack and the servo alone
//     carries the load (motor isn't fighting the latch)
void loadTrebuchet(unsigned long windDurationMs,
                    unsigned long unwindDurationMs,
                    int spoolSpeed = 200) {
  moveSpool(1, windDurationMs, spoolSpeed);       // 1. wind up

  servoAngle(SERVO_LATCH_ANGLE);                  // 2. latch
  delay(300);                                     // let servo finish moving

  moveSpool(-1, unwindDurationMs, spoolSpeed);    // 3. unwind, slack the cable
}

// Drops the latch: servo swings back to 0 and the arm launches.
void releaseTrebuchet() {
  servoAngle(SERVO_RELEASE_ANGLE);
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    initMotor(i);
  }

  ledcAttach(SERVO_PIN, 50, 12);
  servoAngle(SERVO_RELEASE_ANGLE); // start unlatched/safe

  // Example usage:
  // swivelToDirection(DIR_RIGHT);
  // loadTrebuchet(2000, 500);   // wind for 2s, then unwind slack for 0.5s
  // delay(3000);                // wait / aim / whatever
  // releaseTrebuchet();
}

void loop() {
}
