#include "BNO055_support.h"
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <ServoInput.h>
#include <Wire.h>

// #define DEBUG

// This structure contains the details of the BNO055 device that is connected.
// It's updated after initialization.
struct bno055_t myBNO;

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
ServoInputPin<ThrottleSignalPin> rcThrottle(ThrottlePulseMin, ThrottlePulseMax);

const int PitchSignalPin = A0;
const int PitchPulseMin = 950;
const int PitchPulseMax = 2050;
ServoInputPin<PitchSignalPin> rcPitch(PitchPulseMin, PitchPulseMax);

const int RollSignalPin = A2;
const int RollPulseMin = 970;
const int RollPulseMax = 1990;
ServoInputPin<RollSignalPin> rcRoll(RollPulseMin, RollPulseMax);

const int YawSignalPin = A3;
const int YawPulseMin = 970;
const int YawPulseMax = 1960;
ServoInputPin<YawSignalPin> rcYaw(YawPulseMin, YawPulseMax);

#define PITCH 0
#define ROLL 1
#define YAW 2
#define THROTTLE 3

int imu_angle[3];                  // pitch, roll and heading angles
int imu_angle_offset[2] = {-4, 0}; // pitch and roll angle offsets
int imu_rate[3];                   // pitch, roll and yaw angular velocity
int rc_angle[3]; // pitch and roll are angles, but yaw is angular velocity
uint16_t rc_throttle;
bool maintain_heading = false;
int heading = 0;

const float error_sum_limit = 400;
float angle_error[3] = {0, 0, 0};
float rate_error[3];
float acc_error[3];
float previous_rate_error[3] = {0, 0, 0};

/* These values worked outside
float Kp[3] = {1.5, 1.5, 2};
float Ki[3] = {0.5, 0.5, 0.01};
float Kd[3] = {10, 10, 0};
*/
float Kp[3] = {1.5, 1.5, 2};
float Ki[3] = {0.5, 0.5, 0.1};
float Kd[3] = {5, 5, 0};
const uint16_t throttle_threshold = 980;

#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 9
#define MOTOR4_PIN 10

const uint8_t portd_mask =
    (uint8_t)(1 << MOTOR2_PIN) | (uint8_t)(1 << MOTOR1_PIN);
const uint8_t portb_mask =
    (uint8_t)(1 << (MOTOR4_PIN - 8)) | (uint8_t)(1 << (MOTOR3_PIN - 8));

#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000
#define MOTOR_MIN_RUNNING_PULSE 1100
#define MAX_PULSE_CHANGE (MOTOR_MAX_PULSE - MOTOR_MIN_RUNNING_PULSE)

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3

uint16_t motor_pulse[4];

void initImu() {
  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);
}

unsigned long rc_no_signal_cnt[4];
bool emergency_mode;

void initRcControls() {
  ServoInput.attach();
  while (!ServoInput.available()) {
#ifdef DEBUG
    Serial.println("Waiting for servo signals...");
#endif
    delay(100);
  }

  rc_no_signal_cnt[PITCH] = 0;
  rc_no_signal_cnt[ROLL] = 0;
  rc_no_signal_cnt[YAW] = 0;
  rc_no_signal_cnt[THROTTLE] = 0;

  emergency_mode = false;
}

void getImuInputs() {
#ifdef DEBUG
  unsigned long start_us = micros();
#endif

  struct bno055_euler eulerData;
  struct bno055_gyro gyroData;
  if (bno055_read_euler_hrp(&eulerData)) {
#ifdef DEBUG
    Serial.println("Failed reading Euler angles");
#endif
  } else {
    imu_angle[PITCH] = (eulerData.p / 16) - imu_angle_offset[PITCH];
    imu_angle[ROLL] = (eulerData.r / 16) - imu_angle_offset[ROLL];
    imu_angle[YAW] = (eulerData.h / 16);
  }
  if (bno055_read_gyro_xyz(&gyroData)) {
#ifdef DEBUG
    Serial.println("Failed reading yaw rate");
#endif
  } else {
    imu_rate[PITCH] = gyroData.x / 16;
    imu_rate[ROLL] = gyroData.y / 16;
    imu_rate[YAW] = gyroData.z / 16;
  }

#ifdef DEBUG
  unsigned long elapsed = micros() - start_us;
  Serial.print("Pitch angle: ");
  Serial.print(imu_angle[PITCH]);
  Serial.print("    ");
  Serial.print("Roll angle: ");
  Serial.print(imu_angle[ROLL]);
  Serial.print("    ");
  Serial.print("Yaw angle: ");
  Serial.print(imu_angle[YAW]);
  Serial.print("    ");
  Serial.print("Pitch rate: ");
  Serial.print(imu_rate[PITCH]);
  Serial.print("    ");
  Serial.print("Roll rate: ");
  Serial.print(imu_rate[ROLL]);
  Serial.print("    ");
  Serial.print("Yaw rate: ");
  Serial.print(imu_rate[YAW]);
  Serial.print("    ");
  Serial.print("Elapsed time: ");
  Serial.println(elapsed);
#endif
}

// Expected time to go through one loop iteration (10000us)
const unsigned long loop_period = 10000;

// RC signal emergency timeout is set to 10 seconds
const unsigned long rc_signal_timeout_us = 10000000;
const unsigned long rc_signal_emergency_trigger =
    rc_signal_timeout_us / loop_period;

#define EMERGENCY_ANGLE 0
const unsigned long emergency_landing_time_us = 20000000; // 20 seconds
unsigned long emergency_throttle_step_us;
unsigned long emergency_next_throttle_decrease_us;

bool rcSignalEmergency() {
  // If already in emergency mode, no need to check for RC signals
  if (emergency_mode) {
    return true;
  }

  if (rc_no_signal_cnt[THROTTLE] > rc_signal_emergency_trigger ||
      rc_no_signal_cnt[PITCH] > rc_signal_emergency_trigger ||
      rc_no_signal_cnt[ROLL] > rc_signal_emergency_trigger ||
      rc_no_signal_cnt[YAW] > rc_signal_emergency_trigger) {

    // By setting this mode, there's no going back until reset
    emergency_mode = true;
    if (rc_throttle <= MOTOR_MIN_PULSE) {
      emergency_throttle_step_us = loop_period;
    } else {
      emergency_throttle_step_us =
          emergency_landing_time_us / (rc_throttle - MOTOR_MIN_PULSE);
    }
    emergency_next_throttle_decrease_us = micros() + emergency_throttle_step_us;
    return true;
  }

  return false;
}

void getRcControls() {
#ifdef DEBUG
  unsigned long start_us = micros();
#endif

  /**
   * Check every RC channel looking for an invalid signal value
   * which would mean a cable is disconected or cut off.
   */

  // Check THROTTLE signal
  if (!rcThrottle.available()) {
    rc_no_signal_cnt[THROTTLE] += 1;
  } else {
    rc_no_signal_cnt[THROTTLE] = 0;
  }
  // Check PITCH signal
  if (!rcPitch.available()) {
    rc_no_signal_cnt[PITCH] += 1;
  } else {
    rc_no_signal_cnt[PITCH] = 0;
  }
  // Check ROLL signal
  if (!rcRoll.available()) {
    rc_no_signal_cnt[ROLL] += 1;
  } else {
    rc_no_signal_cnt[ROLL] = 0;
  }
  // Check YAW signal
  if (!rcYaw.available()) {
    rc_no_signal_cnt[YAW] += 1;
  } else {
    rc_no_signal_cnt[YAW] = 0;
  }

  rc_throttle = rcThrottle.getPulse();
  rc_angle[PITCH] = rcPitch.mapDeadzone(0, 60, 0.01) - 30;
  rc_angle[ROLL] = rcRoll.mapDeadzone(0, 60, 0.01) - 30;
  rc_angle[YAW] = rcYaw.mapDeadzone(0, 120, 0.1) - 60;

#ifdef DEBUG
  unsigned long elapsed = micros() - start_us;
  Serial.print("RC throttle pulse: ");
  Serial.print(rc_throttle);
  Serial.print("    ");
  Serial.print("RC pitch angle: ");
  Serial.print(rc_angle[PITCH]);
  Serial.print("    ");
  Serial.print("RC roll angle: ");
  Serial.print(rc_angle[ROLL]);
  Serial.print("    ");
  Serial.print("RC yaw rate: ");
  Serial.print(rc_angle[YAW]);
  Serial.print("    ");
  Serial.print("Time elapsed: ");
  Serial.println(elapsed);
#endif
}

void forceRcControlEmergency() {
  if (rc_throttle <= ThrottlePulseMin) {
    rc_throttle = ThrottlePulseMin;
  } else {
    if (micros() > emergency_next_throttle_decrease_us) {
      rc_throttle -= 1;
      emergency_next_throttle_decrease_us += emergency_throttle_step_us;
    }
  }
  rc_angle[PITCH] = EMERGENCY_ANGLE;
  rc_angle[ROLL] = EMERGENCY_ANGLE;
  rc_angle[YAW] = EMERGENCY_ANGLE;
}

float minMax(float value, float min_value, float max_value) {
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }

  return value;
}

void computeErrors() {
  if (rc_angle[YAW] == 0) {
    if (!maintain_heading) {
      maintain_heading = true;
      heading = imu_angle[YAW];
    }
  } else {
    maintain_heading = false;
    heading = 0;
  }

  angle_error[PITCH] = imu_angle[PITCH] - rc_angle[PITCH];
  angle_error[ROLL] = imu_angle[ROLL] - rc_angle[ROLL];

  rate_error[PITCH] = (angle_error[PITCH] * 5) - imu_rate[PITCH];
  rate_error[ROLL] = (angle_error[ROLL] * 5) - imu_rate[ROLL];
  rate_error[YAW] = rc_angle[YAW] - imu_rate[YAW];

  if (maintain_heading) {
    angle_error[YAW] = imu_angle[YAW] - heading;
    if (angle_error[YAW] > 180) {
      angle_error[YAW] -= 360;
    } else if (angle_error[YAW] < -180) {
      angle_error[YAW] += 360;
    }
  } else {
    angle_error[YAW] += rate_error[YAW];
    angle_error[YAW] = minMax(angle_error[YAW], -error_sum_limit / Ki[YAW],
                              error_sum_limit / Ki[YAW]);
  }

  acc_error[PITCH] = rate_error[PITCH] - previous_rate_error[PITCH];
  acc_error[ROLL] = rate_error[ROLL] - previous_rate_error[ROLL];
  acc_error[YAW] = rate_error[YAW] - previous_rate_error[YAW];

  previous_rate_error[PITCH] = rate_error[PITCH];
  previous_rate_error[ROLL] = rate_error[ROLL];
  previous_rate_error[YAW] = rate_error[YAW];

#ifdef DEBUG
  Serial.print("Pitch angle error: ");
  Serial.print(angle_error[PITCH]);
  Serial.print("    ");
  Serial.print("Roll angle error: ");
  Serial.print(angle_error[ROLL]);
  Serial.print("    ");
  Serial.print("Yaw angle error: ");
  Serial.println(angle_error[YAW]);

  Serial.print("Pitch rate error: ");
  Serial.print(rate_error[PITCH]);
  Serial.print("    ");
  Serial.print("Roll rate error: ");
  Serial.print(rate_error[ROLL]);
  Serial.print("    ");
  Serial.print("Yaw rate error: ");
  Serial.println(rate_error[YAW]);

  Serial.print("Pitch acc error: ");
  Serial.print(acc_error[PITCH]);
  Serial.print("    ");
  Serial.print("Roll acc error: ");
  Serial.print(acc_error[ROLL]);
  Serial.print("    ");
  Serial.print("Yaw acc_error: ");
  Serial.println(acc_error[YAW]);
#endif
}

/**
 * Compute motor pulse in microseconds based on PID computation
 *
 * (1) (2)
 *   \ /
 *    X
 *   / \
 * (4) (3)
 *
 * Motors 1 & 3 run clockwise.
 * Motors 2 & 4 run counter-clockwise.
 *
 * Pitch angle is positive when motors 1 & 2 are above 3 & 4
 * Roll angle is positive when motors 1 & 4 are above 2 & 3
 * Pitch rate is positive when motors 1 & 2 are below 3 & 4
 * Roll rate is positive when motors 1 & 4 are below 2 & 3
 * Yaw rate is positive when motor 2 moves towards 1 (2 -> 1, 3 -> 2, 4 -> 3, 1
 * -> 4)
 *
 */
void computeOutputs() {
#ifdef DEBUG
  unsigned long start_us = micros();
#endif
  float pitch_pid, roll_pid, yaw_pid;

  // No need to compute the new outputs if the RC controller didn't give the
  // signal to run.
  if (rc_throttle < throttle_threshold) {
    angle_error[PITCH] = angle_error[ROLL] = angle_error[YAW] = 0;
    previous_rate_error[PITCH] = previous_rate_error[ROLL] =
        previous_rate_error[YAW] = 0;
    motor_pulse[MOTOR1] = motor_pulse[MOTOR2] = motor_pulse[MOTOR3] =
        motor_pulse[MOTOR4] = rc_throttle;
  } else {
    pitch_pid = minMax((angle_error[PITCH] * Ki[PITCH]) +
                           (rate_error[PITCH] * Kp[PITCH]) +
                           (acc_error[PITCH] * Kd[PITCH]),
                       -MAX_PULSE_CHANGE, MAX_PULSE_CHANGE);
    roll_pid =
        minMax((angle_error[ROLL] * Ki[ROLL]) + (rate_error[ROLL] * Kp[ROLL]) +
                   (acc_error[ROLL] * Kd[ROLL]),
               -MAX_PULSE_CHANGE, MAX_PULSE_CHANGE);
    yaw_pid =
        minMax((angle_error[YAW] * Ki[YAW]) + (rate_error[YAW] * Kp[YAW]) +
                   (acc_error[YAW] * Kd[YAW]),
               -MAX_PULSE_CHANGE, MAX_PULSE_CHANGE);

    motor_pulse[MOTOR1] = minMax(rc_throttle - roll_pid - pitch_pid + yaw_pid,
                                 MOTOR_MIN_RUNNING_PULSE, MOTOR_MAX_PULSE);
    motor_pulse[MOTOR2] = minMax(rc_throttle + roll_pid - pitch_pid - yaw_pid,
                                 MOTOR_MIN_RUNNING_PULSE, MOTOR_MAX_PULSE);
    motor_pulse[MOTOR3] = minMax(rc_throttle + roll_pid + pitch_pid + yaw_pid,
                                 MOTOR_MIN_RUNNING_PULSE, MOTOR_MAX_PULSE);
    motor_pulse[MOTOR4] = minMax(rc_throttle - roll_pid + pitch_pid - yaw_pid,
                                 MOTOR_MIN_RUNNING_PULSE, MOTOR_MAX_PULSE);
  }

#ifdef DEBUG
  unsigned long elapsed = micros() - start_us;
  Serial.print("Motor1 pulse: ");
  Serial.print(motor_pulse[MOTOR1]);
  Serial.print("    ");
  Serial.print("Motor2 pulse: ");
  Serial.print(motor_pulse[MOTOR2]);
  Serial.print("    ");
  Serial.print("Motor3 pulse: ");
  Serial.print(motor_pulse[MOTOR3]);
  Serial.print("    ");
  Serial.print("Motor4 pulse: ");
  Serial.print(motor_pulse[MOTOR4]);
  Serial.print("    ");
  Serial.print("Time elapsed: ");
  Serial.println(elapsed);
#endif
}

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

const unsigned long init_esc_time_us = 1000000; // 1 second

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

unsigned long start_loop;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Init IMU
  initImu();

  // Init RC controls
  initRcControls();

  // Init motors
  initMotors();

  start_loop = micros();
}

void loop() {
#ifdef DEBUG
  unsigned long elapsed, start_us = micros();
#endif

  // Retrieve pitch and roll angles, along with yaw rate
  getImuInputs();

  if (rcSignalEmergency()) {
    forceRcControlEmergency();
  } else {
    // Retrieve RC control
    getRcControls();
  }

  // Compute errors
  computeErrors();

  // Compute outputs from PID
  computeOutputs();

  // Adjust the loop timing
  while ((micros() - start_loop) < loop_period)
    ;

  // Reset reference time for the loop
  start_loop = micros();

  // Drive motors according to the computed pulse values
  driveMotors();

#ifdef DEBUG
  elapsed = micros() - start_us;
  Serial.print("Loop elapsed time: ");
  Serial.println(elapsed);
#endif

#ifdef DEBUG
  delay(100);
#endif
}