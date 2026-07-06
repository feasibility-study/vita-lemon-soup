#define SERVO_PIN 44

void servoAngle(int angle) {
  // Convert angle to microseconds (standard servo ranges from 500 to 2500 microsec)
  int pulseWidthMicros = map(angle, 0, 180, 500, 2500);
  
  // Convert microseconds to 12-bit digital value for 50Hz cycle
  // (pulseWidthMicros / 20000) * 4096
  uint32_t duty = ((uint32_t)pulseWidthMicros * 4096UL) / 20000UL;
  
  ledcWrite(SERVO_PIN, duty);
}

void setup() {
  // Attach Pin 44 to a 50Hz frequency with 12-bit resolution (Max for S3 is 14-bit)
  ledcAttach(SERVO_PIN, 50, 12);
  
  // Test angles
  servoAngle(0);
  delay(1000);
  
  servoAngle(90);
  delay(2000);
  
  servoAngle(0);
  delay(1000);
}

void loop() {
}