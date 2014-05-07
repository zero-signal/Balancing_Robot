//#############################################################################
// A Self Balancing Robot
//
// Angle.ino - Angle related functions
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

//#############################################################################
// Angle functions

void update_angle(){
  byte imu_acc_return_code = imu_acc_read();  
  byte imu_gyro_return_code = imu_gyro_read();
  
  /** calculate acc angle **/
  acc_xG = (g_imu_acc_curr->accel_x - zG[0])/256.0;
  acc_yG = (g_imu_acc_curr->accel_y - zG[1])/256.0;
  acc_zG = (g_imu_acc_curr->accel_z - zG[2])/256.0; 
  
  acc_angle = (atan2(acc_xG, sqrt(acc_yG * acc_yG + acc_zG * acc_zG))*180.0)/M_PI;
  
  /** calculate gyro rate **/
  gyro_rate = ((double)g_imu_gyro_curr->gyro_x-gyro_zero)*L3G4200D_DPS_PER_DIGIT;
  
  /* calculate gyro angle */
  gyro_angle = acc_angle;
  gyro_angle += gyro_rate *((double)(micros()-kalman_timer)/1000000.0);
  
  /** calculate fused angle **/
  pitch = kalman.getAngle(acc_angle, gyro_rate, (double)(micros()-kalman_timer)/1000000.0);  
  
  kalman_timer = micros();
}
