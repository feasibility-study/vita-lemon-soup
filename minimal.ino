#define SERVO_PIN 44

void servoAngle(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, pulseWidth * 65536 / 20000);
}

void setup() {
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  ledcAttach(SERVO_PIN, 50, 16);

  
  servoAngle(0);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  delay(5000);

  servoAngle(90);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  delay(2000);

  ledcWrite(SERVO_PIN, 32768);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  delay(10000);

  servoAngle(90);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
}

void loop() {
}
