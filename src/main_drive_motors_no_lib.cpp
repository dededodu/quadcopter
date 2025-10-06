#include <Arduino.h>

/*
#define MOTOR1_PIN 5 
#define MOTOR2_PIN 6 
#define MOTOR3_PIN 9 
#define MOTOR4_PIN 10 
 
void setup() {
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    pinMode(MOTOR3_PIN, OUTPUT);
    pinMode(MOTOR4_PIN, OUTPUT);
}


void loop() {
    digitalWrite(MOTOR1_PIN, HIGH);
    digitalWrite(MOTOR2_PIN, HIGH);
    digitalWrite(MOTOR3_PIN, HIGH);
    digitalWrite(MOTOR4_PIN, HIGH);

    digitalWrite(MOTOR1_PIN, LOW);
    digitalWrite(MOTOR2_PIN, LOW);
    digitalWrite(MOTOR3_PIN, LOW);
    digitalWrite(MOTOR4_PIN, LOW);
}
*/

void setup() {
    DDRB |= 0b00000100;
}

void loop() {
    PORTB |= 0b00000100;
    PORTB &= 0b11111011;
}