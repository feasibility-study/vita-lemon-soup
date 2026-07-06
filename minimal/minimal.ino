/*
   Minimal 5-motor driver sketch for ESP32-S3
   Assumes each motor is controlled by a pair of driver input pins.
*/

#define MOTOR_COUNT 5
#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

// Pin mapping for the 5 motors
const int MOTOR_PINS[MOTOR_COUNT][2] = {
  {1, 2},   // Motor 1 -- Spool to move cable
  {3, 4},   // Motor 2 -- drive base (2-5)
  {5, 6},   // Motor 3
  {7, 8},   // Motor 4
  {9, 44}   // Motor 5 
};

// drivebase is as follows
//      facing right
// facing up    facing down
//      facing left

void initMotor(int motorIndex) {
  int pinA = MOTOR_PINS[motorIndex][0];
  int pinB = MOTOR_PINS[motorIndex][1];

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  
  // Attach pins to the ESP32 LEDC PWM peripheral
  ledcAttach(pinA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinB, PWM_FREQUENCY, PWM_RESOLUTION);

  // Start with motor stopped
  ledcWrite(pinA, 0);
  ledcWrite(pinB, 0);
}

void setMotor(int motorIndex, int speed) {
  // Bound the speed between -255 and 255
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

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("aaa forward");
  initMotor(0);
  setMotor(0, 255);
}

void loop() {
  // Your main logic goes here
  // e.g., setMotor(0, 200); // Drive motor 1 at speed 200
}