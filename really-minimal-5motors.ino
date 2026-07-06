/*
   Minimal 1-motor driver sketch for ESP32-S3
   Controls a single motor on Pins 1 and 2.
*/

#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 8

// Define the pins for Motor 1
const int PIN_A = 1;
const int PIN_B = 2;

void initMotor() {
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  
  // Attach pins to the ESP32 LEDC PWM peripheral
  ledcAttach(PIN_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PIN_B, PWM_FREQUENCY, PWM_RESOLUTION);

  // Start with motor stopped
  ledcWrite(PIN_A, 0);
  ledcWrite(PIN_B, 0);
}

void setMotor(int speed) {
  // Bound the speed between -255 and 255
  speed = constrain(speed, -255, 255);
  int duty = abs(speed);

  if (speed > 0) {
    ledcWrite(PIN_A, duty);
    ledcWrite(PIN_B, 0);
  } else if (speed < 0) {
    ledcWrite(PIN_A, 0);
    ledcWrite(PIN_B, duty);
  } else {
    ledcWrite(PIN_A, 0);
    ledcWrite(PIN_B, 0);
  }
}

void stopMotor() {
  setMotor(0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize our single motor
  initMotor();

  // Quick test routine
  Serial.println("Moving forward...");
  setMotor(150);
  delay(1000);

  Serial.println("Stopping...");
  stopMotor();
  delay(500);

  Serial.println("Moving backward...");
  setMotor(-150);
  delay(1000);

  Serial.println("Stopping...");
  stopMotor();
}

void loop() {
  // Your main logic goes here
  // e.g., setMotor(200);
}