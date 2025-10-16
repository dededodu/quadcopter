#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <ServoInput.h>

const int ThrottleSignalPin = A1;
const int ThrottlePulseMin = 890;
const int ThrottlePulseMax = 1840;
ServoInputPin<ThrottleSignalPin> rcThrottle(ThrottlePulseMin, ThrottlePulseMax);

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3

#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 9
#define MOTOR4_PIN 10

#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000

uint16_t motor_pulse[4];

const uint8_t portd_mask =
    (uint8_t)(1 << MOTOR2_PIN) | (uint8_t)(1 << MOTOR1_PIN);
const uint8_t portb_mask =
    (uint8_t)(1 << (MOTOR4_PIN - 8)) | (uint8_t)(1 << (MOTOR3_PIN - 8));

const unsigned long init_esc_time_us = 3000000; // 3 second

void driveMotors() {
  unsigned long elapsed, start = micros();

  // Set HIGH state on all 4 motors (starting the pulse)
  PORTD |= portd_mask;
  PORTB |= portb_mask;

  while ((PORTD & portd_mask) || (PORTB & portb_mask)) {
    elapsed = micros() - start;

    // Set LOW state to every motor individually depending on expected
    // motor pulse
    if (elapsed >= motor_pulse[MOTOR1]) {
      PORTD &= (uint8_t) ~(1 << MOTOR1_PIN);
    }
    if (elapsed >= motor_pulse[MOTOR2]) {
      PORTD &= (uint8_t) ~(1 << MOTOR2_PIN);
    }
    if (elapsed >= motor_pulse[MOTOR3]) {
      PORTB &= (uint8_t) ~(1 << (MOTOR3_PIN - 8));
    }
    if (elapsed >= motor_pulse[MOTOR4]) {
      PORTB &= (uint8_t) ~(1 << (MOTOR4_PIN - 8));
    }
  }
}

void initRcControls() {
  ServoInput.attach();
  while (!ServoInput.available()) {
    delay(100);
  }
}

void getRcControls() {
  motor_pulse[MOTOR1] = motor_pulse[MOTOR2] = motor_pulse[MOTOR3] =
      motor_pulse[MOTOR4] = rcThrottle.map(MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
}

void initMotors() {
  // Setup motor pin directions
  DDRD |= portd_mask;
  DDRB |= portb_mask;

  // Initialize ESCs
  motor_pulse[MOTOR1] = motor_pulse[MOTOR2] = motor_pulse[MOTOR3] =
      motor_pulse[MOTOR4] = MOTOR_MIN_PULSE;
  unsigned long start = micros();
  while ((micros() - start) < init_esc_time_us) {
    driveMotors();
    delay(10);
  }
}

const unsigned long loop_period_us = 10000; // 10ms
unsigned long start;

void setup() {
  initRcControls();
  initMotors();
  start = micros();
}

void loop() {
  getRcControls();
  while ((micros() - start) < loop_period_us)
    ;
  start = micros();
  driveMotors();
}