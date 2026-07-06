#define SERVO_PIN 44

void setup() {
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  ledcAttach(SERVO_PIN, 50, 16);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  ledcWrite(SERVO_PIN, 32768);
}

void loop() {
}
