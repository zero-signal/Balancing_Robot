//#############################################################################
// A Self Balancing Robot
//
// Balancing_Robot.ino - Main file
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
// A two wheeled self balancing robot using the following hardware:
//    - Arduino Uno (R3)
//    - Arduino Motor Shield (R3)
//    - GY-80 10DOF IMU
//      (L3G4200D Gyro, ADXL345 Acc, HMC5883L Mag, BMP085 Baro/Temp)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

const char * VERSION = "0.1";

//#############################################################################
// General includes
#include <utility/twi.h>
#include <Wire.h>

//#############################################################################
// Third party includes
#include <Kalman.h>

//#############################################################################
// Program specific includes
#include "Balancing_Robot.h"

//#############################################################################
// General variables
boolean debug = false;    // enables debug output via serial interface

//#############################################################################
//#############################################################################
// Main routines (setup and loop)
//#############################################################################
void setup() {
  
  /* Serial setup */
  Serial.begin (115200);
  serial_setup(); 
    
  /* I2C init */
  Wire.begin();
  
  /* IMU init and calibration */
  imu_init();  
  imu_gyro_calibrate();  
    
  /* Configure motors */
  motor_init();  
 
  /* Control timers */
  loop_start_time = micros();  
  kalman_timer = loop_start_time;
  
  serial_in_timer = millis();
  serial_out_timer = serial_in_timer;
}

void loop(){  
  
  update_angle();
  
  update_pid();
  
  update_motors(); 
  
  /* Use a fixed time loop */
  last_loop_time = micros() - loop_start_time;
  if (last_loop_time < STD_LOOP_TIME) {
    while((micros() - loop_start_time) < STD_LOOP_TIME)
      serial_out();
      serial_in();
  }
  loop_start_time = micros();   
}
