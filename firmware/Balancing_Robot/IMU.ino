//#############################################################################
// A Self Balancing Robot
//
// IMU.ino - GY-80 IMU functions
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

//#############################################################################
// GY-80 IMU functions

/*
 * Generic I2C write function
 *
 */
byte imu_write_register(byte device_addr, byte reg_addr, int nbytes, byte *buffer)
{
	byte written_bytes;

	Wire.beginTransmission(device_addr);
	Wire.write(reg_addr);
	written_bytes = Wire.write(buffer, nbytes);
	Wire.endTransmission();

	return written_bytes;
}

/*
 * Generic I2C read function
 *
 */
byte imu_read_register(byte device_addr, byte reg_addr, int nbytes, byte *buffer)
{
	byte read_bytes = 0;

	Wire.beginTransmission(device_addr);
	Wire.write(reg_addr);
	Wire.endTransmission(); 

	Wire.requestFrom((uint8_t)device_addr, (uint8_t)nbytes);

	while(Wire.available() && read_bytes < nbytes)
	{ 
		buffer[read_bytes++] = Wire.read();
	}

	return read_bytes;
}

/*
 * IMU init
 *
 */
byte imu_init(){
  imu_acc_init();
  imu_gyro_init();
}

/*
 * ADXL345 initialisation. Wakes up device and sets resolution to ±2g
 *
 */
byte imu_acc_init(){
  byte rc = 1;

  /* wake up the acc */
  byte data = 0x08;
  byte written = imu_write_register(ADXL345_TWI_ADDR, ADXL345_POWER_CTL, 1, &data);
  
  /* place in ±2g resolution (default, but make sure) */
  data = 0x00;
  written += imu_write_register(ADXL345_TWI_ADDR, ADXL345_DATA_FORMAT, 1, &data);
  
  if (written == 2){
    rc = 0;
  }    
  
  return rc;
}

/*
 * ADXL345 read. Reads 6 bytes of sensor data for X,Y and Z axes.
 *
 */
byte imu_acc_read()
{
  byte rc = 1;
  byte cnt = imu_read_register(ADXL345_TWI_ADDR, ADXL345_DATAX0, ADXL345_TELEGRAM_LEN, g_imu_acc_buf);

  // If we recieved the 6 bytes, then go record them
  if (cnt == ADXL345_TELEGRAM_LEN)  {
    if (g_imu_acc_buf[0] == 0x00 && g_imu_acc_buf[1] == 0x00 && 
       g_imu_acc_buf[2] == 0x00 && g_imu_acc_buf[3] == 0x00 && 
       g_imu_acc_buf[4] == 0x00 && g_imu_acc_buf[5] == 0x00){
         
       rc = 2;
     }
     else if (g_imu_acc_buf[0] == 0xFF && g_imu_acc_buf[1] == 0xFF && 
       g_imu_acc_buf[2] == 0xFF && g_imu_acc_buf[3] == 0xFF && 
       g_imu_acc_buf[4] == 0xFF && g_imu_acc_buf[5] == 0xFF) {
       
       rc = 3;
     }
     else {
       g_imu_acc_seq++;
       
       // Unpack ADXL345 accelerometer data
       g_imu_acc_curr->accel_x    = -(g_imu_acc_buf[0] + (g_imu_acc_buf[1] << 8));
       g_imu_acc_curr->accel_y    = g_imu_acc_buf[2] + (g_imu_acc_buf[3] << 8);
       g_imu_acc_curr->accel_z    = g_imu_acc_buf[4] + (g_imu_acc_buf[5] << 8);
       
       rc = 0;
     }     
  }

  return rc;
}

/*
 * L3G4200D initialisation. Turns on device and sets scale to be 250 deg/sec.
 *
 */
byte imu_gyro_init(){
  byte rc = 1;

  byte data = 0x1F;          // Turn on all axes, disable power down
  byte written = imu_write_register(L3G4200D_TWI_ADDR, L3G4200D_CTRL_REG1, 1, &data);
  
  data = 0x08;               // Enable control ready signal
  written += imu_write_register(L3G4200D_TWI_ADDR, L3G4200D_CTRL_REG3, 1, &data);
  
  data = 0x80;               // Set scale (250 deg/sec)
  written += imu_write_register(L3G4200D_TWI_ADDR, L3G4200D_CTRL_REG4, 1, &data);
        
  if (written == 3){
    rc = 0;
  } 

  return rc;  
}

/*
 * L3G4200D read. Reads 6 bytes of sensor data for X,Y and Z axes.
 *
 */
byte imu_gyro_read(){
  byte rc = 1;
  byte cnt = 0; 
  
  byte reg = L3G4299D_OUT_X_L;
  uint8_t *buf = g_imu_gyro_buf;
  
  // seems like we cant read across registers with this gyro :-(
  for (byte j=0;j<L3G4200D_TELEGRAM_LEN;j++)  {  
      cnt += imu_read_register(L3G4200D_TWI_ADDR, reg++, 1, buf++);
  }
  
  // If we recieved the 6 bytes, then go record them
  if (cnt == L3G4200D_TELEGRAM_LEN)  {
    if (g_imu_gyro_buf[0] == 0x00 && g_imu_gyro_buf[1] == 0x00 && 
       g_imu_gyro_buf[2] == 0x00 && g_imu_gyro_buf[3] == 0x00 && 
       g_imu_gyro_buf[4] == 0x00 && g_imu_gyro_buf[5] == 0x00){
         
       rc = 2;
     }
     else if (g_imu_gyro_buf[0] == 0xFF && g_imu_gyro_buf[1] == 0xFF && 
       g_imu_gyro_buf[2] == 0xFF && g_imu_gyro_buf[3] == 0xFF && 
       g_imu_gyro_buf[4] == 0xFF && g_imu_gyro_buf[5] == 0xFF) {
       
       rc = 3;
     }
     else {
       g_imu_gyro_seq++;
       
       // Unpack L3G4200D accelerometer data
       g_imu_gyro_curr->gyro_x    = g_imu_gyro_buf[2] + (g_imu_gyro_buf[3] << 8);
       g_imu_gyro_curr->gyro_y    = g_imu_gyro_buf[0] + (g_imu_gyro_buf[1] << 8);
       g_imu_gyro_curr->gyro_z    = g_imu_gyro_buf[4] + (g_imu_gyro_buf[5] << 8);
       
       rc = 0;
     }     
  }

  return rc;
}

/*
 * Gyro calibration routine. Averages L3G4200D_NUM_SAMPLES reads from Gyro.
 *
 */
void imu_gyro_calibrate(){                 
  uint16_t gyro_y_buffer[L3G4200D_NUM_SAMPLES];     // holds all our sample values
  uint8_t  gyro_buf[2];                             // temporary buffer to hold raw gyro reading
  
  /* take L3G4200D_NUM_SAMPLES from the gyro */
  for (byte j=0;j<L3G4200D_NUM_SAMPLES;j++)  {  
       uint8_t  *buf = gyro_buf;
       byte reg = L3G4299D_OUT_Y_L;                 // it's actually the Y axis we need in our orientation
       
       for (byte i=0;i<2;i++){
          imu_read_register(L3G4200D_TWI_ADDR, reg++, 1, buf++);
          delay(5);
       }
       gyro_y_buffer[j] = gyro_buf[0] + (gyro_buf[1] << 8);
  }
  
  for (byte i=0;i<L3G4200D_NUM_SAMPLES;i++){
    gyro_zero += gyro_y_buffer[i];
  }
  gyro_zero /= L3G4200D_NUM_SAMPLES;
}

/* Probably need and acc calibration routine somewhere in here too.... */

// never gonna use it...
byte imu_baro_init(unsigned short timeout){
  
}

// might use it...
byte imu_mag_init(unsigned short timeout){
  
}
