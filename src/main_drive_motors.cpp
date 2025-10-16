#include <Arduino.h>
#include <Servo.h>

#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 9
#define MOTOR4_PIN 10

#define MIN_ATTACH_PIN 1000
#define MAX_ATTACH_PIN 2000

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  Serial.begin(115200);
  motor1.attach(MOTOR1_PIN, MIN_ATTACH_PIN, MAX_ATTACH_PIN);
  motor2.attach(MOTOR2_PIN, MIN_ATTACH_PIN, MAX_ATTACH_PIN);
  motor3.attach(MOTOR3_PIN, MIN_ATTACH_PIN, MAX_ATTACH_PIN);
  motor4.attach(MOTOR4_PIN, MIN_ATTACH_PIN, MAX_ATTACH_PIN);
  delay(10);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(1000);
}

void loop() {
  unsigned long elapsed, start = micros();
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  elapsed = micros() - start;
  Serial.print("Time elapsed: ");
  Serial.println(elapsed);
  delay(1000);
  motor1.writeMicroseconds(1100);
  motor2.writeMicroseconds(1100);
  motor3.writeMicroseconds(1100);
  motor4.writeMicroseconds(1100);
  delay(1000);
}