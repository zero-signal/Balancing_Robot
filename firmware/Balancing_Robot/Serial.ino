//#############################################################################
// A Self Balancing Robot
//
// Serial.ino - Serial I/O functions
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

//#############################################################################
// Serial functions

#define D2I(d) (long)(d * 10000)

/*
 * Initialise serial input/output
 */
void serial_setup(){
  
  /* initialise our output format string */
  memset(fmt,0,FORMAT_STR_LEN);
  
  int i;
  char d[] = "%ld,";
  for(i = 0; i < 21; i++){
    strncat(fmt,d,sizeof(d));
  }
  strncat(fmt,"%d,%d",6);  
}

/*
 * Helper function to build a string given a buffer, a format string and a variable arg list
 */
uint8_t build_string(char *buf, const char *fmt, ...){
  uint8_t n = 1;
  va_list ap;
  
  memset(buf,0,SERIAL_WRITE_BYTES);
  
  va_start(ap,fmt);
  n = vsnprintf(buf, SERIAL_WRITE_BYTES, fmt, ap); 
  va_end(ap);
  
  if(n > -1 && n < SERIAL_WRITE_BYTES){
    return 0;
  }
  else {
    return 1;
  }
}

/*
 * Write serial data
 */
void serial_out() { 
  if((millis() - serial_out_timer) >= serial_out_update) { 
    
    uint8_t result = 1;
    result = build_string(serial_out_buf, fmt,
                              D2I(acc_angle),
                              D2I(gyro_angle),
                              D2I(gyro_rate),
                              D2I(pitch),
                              D2I(g_imu_acc_curr->accel_x),
                              D2I(g_imu_acc_curr->accel_y),
                              D2I(g_imu_acc_curr->accel_z),
                              D2I(g_imu_gyro_curr->gyro_x),
                              D2I(g_imu_gyro_curr->gyro_y),
                              D2I(g_imu_gyro_curr->gyro_z),
                              D2I(pid_value), D2I(pT), D2I(iT), D2I(dT),
                              D2I(error), D2I(target_angle), D2I(p), D2I(i), D2I(d),
                              D2I(motor_gain_left),
                              D2I(motor_gain_right),
                              (disabled) ? (int) 1 : (int) 0,
                              (brake) ? (int) 1 : (int) 0
                            );
    
    /* print the resulting buffer */                       
    if(!result){
      Serial.println(serial_out_buf);
    }   
    
    serial_out_timer = millis();
  }
}

/*
 * Read serial data - we read whats there and if its a correctly terminated string
 * parse it to grab the new values. Otherwise, drop it and move on.
 */
void serial_in(){  
  if((millis() - serial_in_timer) >= serial_in_update) {   
    
    /* see what is there */
    int av = Serial.available();
    if(((av > 0) && (av < SERIAL_READ_BYTES)) && (Serial.readBytes(serial_in_buf, av) == av)){
      if(serial_in_buf[av - 1] == '\n'){
         
         serial_in_buf[av - 1] = '\0';
        
          char* s; 
          char* buf = (char *) serial_in_buf;    
          double vals[5];     
        
          int c = 0;
          while((s = strtok(buf,",")) != NULL && c < 5) {
            buf = NULL;
            vals[c] = strtod(s, NULL);
            c++;
          }
          
          /* assign new PID/Gain values */
          if(c == 5){            
            p = (double) vals[0];
            i = (double) vals[1];
            d = (double) vals[2]; 
    
            motor_gain_left = (double) vals[3];   
            motor_gain_right = (double) vals[4];  
 
           Serial.println(p);
          }         
      }
    }
    serial_in_timer = millis();    
  }
}
