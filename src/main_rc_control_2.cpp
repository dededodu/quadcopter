#include <PinChangeInterrupt.h>
#include <ServoInput.h>

/**
 * Default values:
 * Throttle = 896 / Pitch = 1500 / Roll = 1480 / Yaw = 1464
 * Minimum values:
 * Throttle = 888 / Pitch = 956 / Roll = 1056 / Yaw = 992
 * Maximum values:
 * Throttle = 1844 / Pitch = 1952 / Roll = 1996 / Yaw = 1964
 */
const int ThrottleSignalPin = A1;
const int ThrottlePulseMin = 890;
const int ThrottlePulseMax = 1840;
ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

const int PitchSignalPin = A0;
const int PitchPulseMin = 950;
const int PitchPulseMax = 2050;
ServoInputPin<PitchSignalPin> pitch(PitchPulseMin, PitchPulseMax);

const int RollSignalPin = A2;
const int RollPulseMin = 970;
const int RollPulseMax = 1990;
ServoInputPin<RollSignalPin> roll(RollPulseMin, RollPulseMax);

const int YawSignalPin = A3;
const int YawPulseMin = 970;
const int YawPulseMax = 1960;
ServoInputPin<YawSignalPin> yaw(YawPulseMin, YawPulseMax);

void setup() {
  Serial.begin(115200);

  ServoInput.attach();

	while (!ServoInput.available()) {
		Serial.println("Waiting for servo signals...");
		delay(500);
	}
}

void loop() {
  unsigned long start = micros();
  Serial.print("Throttle: ");
  Serial.print(throttle.getPercent() * 100);
  Serial.print("    ");
  Serial.print("Pitch: ");
  Serial.print(pitch.mapDeadzone(0, 60, 0.01) - 30);
  Serial.print("    ");
  Serial.print("Roll: ");
  Serial.print(roll.mapDeadzone(0, 60, 0.01) - 30);
  Serial.print("    ");
  Serial.print("Yaw: ");
  Serial.print((yaw.mapDeadzone(0, 60, 0.01) - 30));
  Serial.print("    ");
  Serial.print("Time elapsed: ");
  Serial.println(micros()-start);

  delay(10);
}