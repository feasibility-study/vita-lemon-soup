#define SERVO_PIN 44
#define MOTOR_COUNT 5
#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

void servoAngle(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, pulseWidth * 4096UL / 20000UL);
}

const int MOTOR_PINS[MOTOR_COUNT][2] = {
  {1, 2},   // Motor 1 -- Spool to move cable
  {3, 4},   // Motor 2 -- drive base (2-5)
  {5, 6},   // Motor 3
  {7, 8},   // Motor 4
  {9, 44}   // Motor 5 
};

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
  initMotor(0);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  ledcAttach(SERVO_PIN, 50, 12);

  
  servoAngle(90);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  setMotor(0, -255);
  delay(5000);
  servoAngle(0);
  stopMotor(0);

  setMotor(0, 255);
  delay(4000);
  servoAngle(90);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
}

void loop() {
}
