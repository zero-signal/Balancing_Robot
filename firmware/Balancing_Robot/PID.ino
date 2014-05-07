//#############################################################################
// A Self Balancing Robot
//
// PID.ino - PID related functions
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

//#############################################################################
// PID functions

void update_pid(){
  error = ((double) target_angle - pitch);
  pT = p * error;
  
  integrated_error += error;
  integrated_error = constrain(integrated_error, -20.0, 20.0);

  iT = i * integrated_error; 
  dT = d * (error - last_error);
  
  last_error = error;
  pid_value = pT + iT + dT;
  
  pid_left  = pid_value;
  pid_right = pid_value; 
}
