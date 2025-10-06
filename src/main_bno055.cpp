#include <Arduino.h>

#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_linear_accel myLinearAccelData;

unsigned long lastTime = 0;

void setup()
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
}

void loop()
{
  unsigned long start = micros();

  bno055_read_euler_hrp(&myEulerData);
  Serial.print("Heading(Yaw): ");
  Serial.print(float(myEulerData.h) / 16.00);
  Serial.print("    ");
  Serial.print("Roll: ");
  Serial.print(float(myEulerData.r) / 16.00);
  Serial.print("    ");
  Serial.print("Pitch: ");
  Serial.print(float(myEulerData.p) / 16.00);
  Serial.print("    ");
  Serial.print("Elapsed time: ");
  Serial.println(micros() - start);

  start = micros();

  bno055_read_linear_accel_xyz(&myLinearAccelData);
  Serial.print("LinearAccel X: ");
  Serial.print(myLinearAccelData.x);
  Serial.print("    ");
  Serial.print("LinearAccel Y: ");
  Serial.print(myLinearAccelData.y);
  Serial.print("    ");
  Serial.print("LinearAccel Z: ");
  Serial.print(myLinearAccelData.z);
  Serial.print("    ");
  Serial.print("Elapsed time: ");
  Serial.println(micros() - start);

  delay(100);
}