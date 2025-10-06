#include <Arduino.h>

#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

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
  
  bno055_set_accel_an_nm_axis_enable(BNO055_ACCEL_AM_NM_X_AXIS, 1);
  bno055_set_accel_an_nm_axis_enable(BNO055_ACCEL_AM_NM_Y_AXIS, 1);
  bno055_set_accel_an_nm_axis_enable(BNO055_ACCEL_AM_NM_Z_AXIS, 1);
  //bno055_set_accel_anymotion_threshold(1);
  //bno055_set_accel_anymotion_duration(1);
  //bno055_set_int_accel_anymotion(1);
  //bno055_set_accel_slow_no_duration(1);
  bno055_set_accel_slow_no_threshold(1);
  bno055_set_accel_slow_no_motion_enable(0);
  bno055_set_int_accel_nomotion(1);
}

void loop()
{
  unsigned long start = micros();
  unsigned char x_mov = 0;
  unsigned char y_mov = 0;
  unsigned char z_mov = 0;
  //bno055_get_interrupt_status_accel_anymotion(&x_mov);
  //bno055_get_interrupt_status_accel_anymotion(&y_mov);
  //bno055_get_interrupt_status_accel_anymotion(&z_mov);
  bno055_get_interrupt_status_accel_nomotion(&x_mov);
  bno055_get_interrupt_status_accel_nomotion(&y_mov);
  bno055_get_interrupt_status_accel_nomotion(&z_mov);
  Serial.print("X movement: ");
  Serial.print(x_mov);
  Serial.print("    ");
  Serial.print("Y movement: ");
  Serial.print(y_mov);
  Serial.print("    ");
  Serial.print("Z movement: ");
  Serial.print(z_mov);
  Serial.print("    ");
  Serial.print("Elapsed time: ");
  Serial.println(micros() - start);

  delay(10);
}