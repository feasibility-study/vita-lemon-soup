#define SERVO_PIN 44

void setup() {
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  ledcAttach(SERVO_PIN, 50, 16);
}

void setAngle(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  ledcWrite(SERVO_PIN, pulseWidth * 65536 / 20000);
}

void loop() {
  setAngle(90);
  delay(1000);
  setAngle(0);
  delay(1000);
}
