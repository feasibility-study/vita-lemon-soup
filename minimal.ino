#include <Servo.h>

Servo servo;

void setup() {
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  servo.attach(44);

  
    servo.write(0);
    digitalWrite(1, HIGH);
    digitalWrite(2, LOW);
    delay(5000);
    servo.write(90);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    delay(2000);
    digitalWrite(1, LOW);
    digitalWrite(2, HIGH);
    delay(10000);
}

void loop() {

}
