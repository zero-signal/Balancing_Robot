//#############################################################################
// A Self Balancing Robot
//
// Motor.ino - Motor functions
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

//#############################################################################
// Motor functions
void motor_init(){

  // initialization of pins  
  pinMode(LEFT_MOTOR_DIR,  OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);

  pinMode(LEFT_MOTOR_PWM,  OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); 

  pinMode(LEFT_MOTOR_BRK,  OUTPUT);
  pinMode(RIGHT_MOTOR_BRK, OUTPUT);   

  pinMode(LEFT_MOTOR_CNT,  INPUT);
  pinMode(RIGHT_MOTOR_CNT, INPUT); 

  // establish motor directions and disengage brake
  digitalWrite(LEFT_MOTOR_DIR,  LEFT_MOTOR_FORWARD);
  digitalWrite(RIGHT_MOTOR_DIR, RIGHT_MOTOR_FORWARD);
  
  /* disengage to be safe */
  stop_motors();
}

void update_motors(){
  if(abs(pitch) > max_angle){   
    stop_motors();
  } else {  
    reset_motors();
    if (pid_value >= 0){
      digitalWrite(LEFT_MOTOR_DIR,  LEFT_MOTOR_BACKWARD);  
      digitalWrite(RIGHT_MOTOR_DIR,  RIGHT_MOTOR_BACKWARD);
    }
    else {
      digitalWrite(LEFT_MOTOR_DIR,  LEFT_MOTOR_FORWARD);   
      digitalWrite(RIGHT_MOTOR_DIR,  RIGHT_MOTOR_FORWARD);    
    }
    
    uint16_t l_pwm = abs(pid_value) * motor_gain_left;
    uint16_t r_pwm = abs(pid_value) * motor_gain_right;
    
    if(!debug){
      analogWrite(LEFT_MOTOR_PWM, l_pwm);
      analogWrite(RIGHT_MOTOR_PWM, r_pwm);
    }
  }
}

void stop_motors(){
  disabled = true;
  brake = true;
    
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  
  digitalWrite(LEFT_MOTOR_BRK,  HIGH);
  digitalWrite(RIGHT_MOTOR_BRK, HIGH);
  
  last_error = integrated_error = error = 0.0;
}

void reset_motors(){
  disabled = false;
  brake = false;  
  
  digitalWrite(LEFT_MOTOR_BRK,  LOW);
  digitalWrite(RIGHT_MOTOR_BRK, LOW);  
}

