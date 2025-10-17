#include <PinChangeInterrupt.h>

#define THROTTLE_PIN A1
#define PITCH_PIN A0
#define ROLL_PIN A2
#define YAW_PIN A3

unsigned long rcThrottleStart;
unsigned long rcPitchStart;
unsigned long rcRollStart;
unsigned long rcYawStart;

volatile unsigned long rcThrottle;
volatile unsigned long rcPitch;
volatile unsigned long rcRoll;
volatile unsigned long rcYaw;

void updateRcThrottle() {
  if (digitalRead(THROTTLE_PIN) == HIGH) {
    rcThrottleStart = micros();
  } else {
    rcThrottle = micros() - rcThrottleStart;
  }
}

void updateRcPitch() {
  if (digitalRead(PITCH_PIN) == HIGH) {
    rcPitchStart = micros();
  } else {
    rcPitch = micros() - rcPitchStart;
  }
}

void updateRcRoll() {
  if (digitalRead(ROLL_PIN) == HIGH) {
    rcRollStart = micros();
  } else {
    rcRoll = micros() - rcRollStart;
  }
}

void updateRcYaw() {
  if (digitalRead(YAW_PIN) == HIGH) {
    rcYawStart = micros();
  } else {
    rcYaw = micros() - rcYawStart;
  }
}

void setup() {
  Serial.begin(115200);
  attachPCINT(digitalPinToPCINT(THROTTLE_PIN), updateRcThrottle, CHANGE);
  attachPCINT(digitalPinToPCINT(PITCH_PIN), updateRcPitch, CHANGE);
  attachPCINT(digitalPinToPCINT(ROLL_PIN), updateRcRoll, CHANGE);
  attachPCINT(digitalPinToPCINT(YAW_PIN), updateRcYaw, CHANGE);
}

void loop() {
  unsigned long start = micros();
  Serial.print("Throttle: ");
  Serial.print(rcThrottle);
  Serial.print("    ");
  Serial.print("Pitch: ");
  Serial.print(rcPitch);
  Serial.print("    ");
  Serial.print("Roll: ");
  Serial.print(rcRoll);
  Serial.print("    ");
  Serial.print("Yaw: ");
  Serial.print(rcYaw);
  Serial.print("    ");
  Serial.print("Time elapsed: ");
  Serial.println(micros() - start);

  delay(100);
}