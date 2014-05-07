//#############################################################################
// A Self Balancing Robot GUI control application
//
// Balancing_Robot_GUI.pde - Main file
//
// Copyright (C) 2014 - Zerosignal (zerosignal1982@gmail.com)
//
// GUI Control program for a two wheeled self balancing robot using the following 
// hardware:
//    - Arduino Uno (R3)
//    - Arduino Motor Shield (R3)
//    - GY-80 10DOF IMU
//      (L3G4200D Gyro, ADXL345 Acc, HMC5883L Mag, BMP085 Baro/Temp)
//
// Makes heavy use of code originally developed by the Open Source Multiwii 
// multirotor control firmware. Details at: https://code.google.com/p/multiwii/
//
//#############################################################################
// Version history:
//   2014-04-24    0.1    Alpha version
//
//#############################################################################

import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 
import processing.serial.Serial; 
import processing.opengl.*; 
import controlP5.*; 

static final float version = 0.1f;

/* variables */
int hPortList  = 80;    int wPortList  = 110;
int hBaudList  = 100;   int wBaudList  = 110;
int hComButton = 20;    int wComButton = 110;  
int hAngGraph  = 400;   int wAngGraph  = 430;
int hPidGraph  = 400;   int wPidGraph  = 430;
int hInd       = 50;    int wInd       = 110;
int hWrite     = 20;    int wWrite     = 110;
int hIndAng    = 40;    int wIndAng    = 430;
int hIndPid    = 40;    int wIndPid    = 430;

int xWindow    = 990;   int yWindow    = 550;
int xPortList  = 5;     int yPortList  = 20;   
int xBaudList  = 5;     int yBaudList  = (yPortList + hPortList)  + 5;
int xStatusLbl = 5;     int yStatusLbl = (yBaudList + hBaudList) + 10;
int xComButton = 5;     int yComButton = (yStatusLbl + 20);
int xAngGraph  = 120;   int yAngGraph  = 10;
int xPidGraph  = 555;   int yPidGraph  = 10;
int xInd       = 5;     int yInd       = (yComButton + hComButton) + 10;
int xWrite     = 5;     int yWrite     = (yInd + hInd) + 10;
int xIndAng    = 125;   int yIndAng    = 415;
int xIndPid    = 560;   int yIndPid    = 415;
int xAngSlider = 120;   int yAngSlider = 465;
int xMotSlider = 555;   int yMotSlider = 465;
int xToggle    = 700;   int yToggle    = 400;
int xLbl       = 200;   int yLbl       = 100;

/* GUI controls */
ControlP5 controlP5;
Textlabel statusTxtLbl, angGraphLbl, pidGraphLbl;
ListBox   commListbox,baudListbox;
Button    commConnectButton, anglePauseButton, pidPauseButton, writeButton;
Slider    scaleSliderAng, scaleSliderPid, pSlider, iSlider, dSlider, lftMotorSlider, rgtMotorSlider;
Toggle    accAngToggle, gyroAngToggle, pitchAngToggle, errorAngToggle, spAngToggle, pValToggle, pTValToggle, iTValToggle, dTValToggle, lpValToggle, rpValToggle, disabledToggle, brakeToggle;

Graph angleGraph, pidGraph;

/* status flags */
boolean disabled = false, brake = false;

/* serial data variables */
float acc_angle, gyro_angle, gyro_rate, pitch, accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z, pid_value, pT, iT, dT, error, target, p, i, d;
float motor_gain_left, motor_gain_right;

/* flags for graph output control */
boolean accAngShow = true, gyroAngShow = true, pitchAngShow = true, errorAngShow = true, spAngShow = true;
boolean pValShow = true, pTValShow = true, iTValShow = true, dTValShow = true, lpValShow = true, rpValShow = true;
boolean anglePause = false, pidPause = false; 

/* comm port flags*/
boolean commSelected = false, baudSelected = false, commConnected = false, serial_read = false;
int commPort = 0; int baudRate = 0;

/* timers */
int time = 0; int update_timer = 0;

/* colors */
color yellow_ = color(255, 255, 0), green_ = color(30, 120, 30), red_ = color(120, 30, 30), blue_ = color(2, 52, 77), 
      purple_ = color(128,0,255), grey_ = color(30, 30, 30), black_ = color(0, 0, 0), orange_ = color(200,128,0),
      r_red_ = color(255,0,0), r_blue_ = color(0,0,255), r_green_ = color(0, 255, 0), r_orange_ = color(255,128,0);

int commListMax = 0;

/* data arrays */
DataArray AccAng   = new DataArray(200);
DataArray GyroAng  = new DataArray(200);
DataArray GyroRate = new DataArray(200);   // we do not graph this yet
DataArray PitchAng = new DataArray(200);
DataArray ErrorAng = new DataArray(200);
DataArray SetAng   = new DataArray(200);

DataArray TotPid   = new DataArray(200);
DataArray PtPid    = new DataArray(200);
DataArray ItPid    = new DataArray(200);
DataArray DtPid    = new DataArray(200);

Serial serial;

public void setup() {
  size(xWindow,yWindow,OPENGL);
  frameRate(20); 

  controlP5 = new ControlP5(this); // initialize the GUI controls
  
  /* available serial ports */
  commListbox = controlP5.addListBox("commListbox", xPortList, yPortList, wPortList, hPortList); 
  commListbox.captionLabel().set("Serial Port");
  for( int i = 0; i < Serial.list().length; i++) {
    String pn = Serial.list()[i];
    
    if (pn.length() > 0 ) {
      commListbox.addItem(pn, i); 
    }
    commListMax = i;
  }
  
  /* baud rate */
  baudListbox = controlP5.addListBox("baudListbox", xBaudList, yBaudList, wBaudList, hBaudList);
  baudListbox.captionLabel().set("Baud Rate");
  
  /* selectable baud rates */
  baudListbox.addItem("9600"  ,9600);
  baudListbox.addItem("14400" ,14400);
  baudListbox.addItem("19200" ,19200);
  baudListbox.addItem("28800" ,28800);
  baudListbox.addItem("38400" ,38400);
  baudListbox.addItem("57600" ,57600);
  baudListbox.addItem("115200",115200);  
  
  /* labels for serial port status information */
  statusTxtLbl = controlP5.addTextlabel("statusTxtLbl", "No port selected", xStatusLbl, yStatusLbl);
  
  /* button for connecting/disconnecting the serial port */
  commConnectButton = controlP5.addButton("commConnectButton", 1, xComButton, yComButton, wComButton, hComButton).setLabel("         Serial Connect");
  
  /* graph scaling slider */
  scaleSliderAng = controlP5.addSlider("scaleSliderAng", 0, 10, 1.0f, (xAngGraph + wAngGraph) - 170, yAngGraph + 10, 100, 15).setLabel("Angle Scale");  
  scaleSliderPid = controlP5.addSlider("scaleSliderPid", 0, 10, 1.0f, (xPidGraph + wPidGraph) - 155, yPidGraph + 10, 100, 15).setLabel("PID Scale");
  
  /* PID control sliders */
  pSlider = controlP5.addSlider("pSlider", 0, 20, 0.0f, xAngSlider, yAngSlider, 200, 15).setLabel("P"); 
  iSlider = controlP5.addSlider("iSlider", 0, 20, 0.0f, xAngSlider, yAngSlider + 20, 200, 15).setLabel("I"); 
  dSlider = controlP5.addSlider("dSlider", 0, 20, 0.0f, xAngSlider, yAngSlider + 40, 200, 15).setLabel("D");  
  
  /* Motor gain sliders */
  lftMotorSlider = controlP5.addSlider("lftMotorSlider", 0.0f, 2.0f, 1.0f, xMotSlider, yMotSlider, 200, 15).setLabel("Left Gain"); 
  rgtMotorSlider = controlP5.addSlider("rgtMotorSlider", 0.0f, 2.0f, 1.0f, xMotSlider, yMotSlider + 20, 200, 15).setLabel("Right Gain"); 
  
  /* Angle + PID Graph objects */
  angleGraph  = new Graph(xAngGraph, yAngGraph, wAngGraph, hAngGraph);
  pidGraph    = new Graph(xPidGraph, yPidGraph, wPidGraph, hPidGraph); 
  
  /* Angle + PID Graph Labels */
  angGraphLbl = controlP5.addTextlabel("angGraphLbl", "Angle", xAngGraph + 10, yAngGraph + 10).setFont(new ControlFont(createFont("Sansserif",18,false)));
  pidGraphLbl = controlP5.addTextlabel("pidGraphLbl", "PID", xPidGraph + 10, yPidGraph + 10).setFont(new ControlFont(createFont("Sansserif",18,false)));
  
  /* Indicator toggles */
  disabledToggle = controlP5.addToggle("disabledToggle", true, xInd + 20, yInd + 10, 20, 20).setCaptionLabel("Disabled").setColorActive(color(255,0,0)).setLock(true);
  brakeToggle    = controlP5.addToggle("brakeToggle", true, xInd + 65, yInd + 10, 20, 20).setCaptionLabel("Brake").setColorActive(color(255,0,0)).setLock(true);
  
  /* Write Button */
  writeButton = controlP5.addButton("writeButton", 1.0f, xWrite, yWrite, wWrite, hWrite).setLabel("           Write Values");
 
  /* Angle Graph controls*/
  accAngToggle = controlP5.addToggle("accAngToggle", accAngShow, xIndAng + 5, yIndAng + 7, 15, 15).setCaptionLabel("Acc").setColorActive(r_red_);
  gyroAngToggle = controlP5.addToggle("gyroAngToggle", gyroAngShow, xIndAng + 45, yIndAng + 7, 15, 15).setCaptionLabel("Gyro").setColorActive(r_orange_);
  pitchAngToggle = controlP5.addToggle("pitchAngToggle", pitchAngShow, xIndAng + 85, yIndAng + 7 , 15, 15).setCaptionLabel("Pitch").setColorActive(yellow_);
  errorAngToggle = controlP5.addToggle("errorAngToggle", errorAngShow, xIndAng + 125, yIndAng + 7, 15, 15).setCaptionLabel("Error").setColorActive(r_green_);
  spAngToggle = controlP5.addToggle("spAngToggle", spAngShow, xIndAng + 165, yIndAng + 7, 15, 15).setCaptionLabel("Setpoint").setColorActive(r_blue_);   
  
  anglePauseButton = controlP5.addButton("aPause", 1.0f, (xIndAng + wIndAng) - 55, yIndAng + 12, 45, 15).setLabel("   Pause");  
 
  /* PID Graph controls */
  pValToggle = controlP5.addToggle("pValToggle", pValShow, xIndPid + 5, yIndPid + 7, 15, 15).setCaptionLabel("PID").setColorActive(r_red_);
  pTValToggle = controlP5.addToggle("pTValToggle", rpValShow, xIndPid + 45, yIndPid + 7, 15, 15).setCaptionLabel("pT PID").setColorActive(r_orange_); 
  iTValToggle = controlP5.addToggle("iTValToggle", rpValShow, xIndPid + 85, yIndPid + 7, 15, 15).setCaptionLabel("iT PID").setColorActive(yellow_); 
  dTValToggle = controlP5.addToggle("dTValToggle", rpValShow, xIndPid + 125, yIndPid + 7, 15, 15).setCaptionLabel("dT PID").setColorActive(r_green_); 
  lpValToggle = controlP5.addToggle("lpValToggle", lpValShow, xIndPid + 165, yIndPid + 7, 15, 15).setCaptionLabel("Lt PID").setColorActive(r_blue_); 
  rpValToggle = controlP5.addToggle("rpValToggle", rpValShow, xIndPid + 205, yIndPid + 7, 15, 15).setCaptionLabel("Rt PID").setColorActive(purple_); 
  
  pidPauseButton   = controlP5.addButton("pPause", 1.0f, (xIndPid + wIndPid) - 55, yIndPid + 12, 45, 15).setLabel("   Pause");
}

public void draw() {
  background(100);
  
  /* Indicator box */
  stroke(0, 0, 0);
  strokeWeight(1);
    
  fill(145);
    
  rectMode(CORNER);
  rect(xInd, yInd, wInd, hInd);
  rect(xIndPid, yIndPid, wIndPid, hIndPid);   
  rect(xIndAng, yIndAng, wIndAng, hIndAng);
  
  /* Text info */
  fill(255);
  text("Balancing Robot GUI v" + version, width - 160, height - 10);  
  
  /* draw graph boundaries */
  angleGraph.drawGraphBox();  
  pidGraph.drawGraphBox();
  
  if(commConnected){
    time = millis();
    
    /* update every 50ms */
    if((time - update_timer) > 40) {
      /* Angle */
      if(!anglePause){
        AccAng.addVal(acc_angle);
        GyroAng.addVal(gyro_angle);
        PitchAng.addVal(pitch);     
        SetAng.addVal(target); 
        ErrorAng.addVal(error);
      }
      
      /* PID */
      TotPid.addVal(pid_value);
      PtPid.addVal(pT);
      ItPid.addVal(iT);
      DtPid.addVal(dT);
      
      /* draw graph lines - angle graph */
      strokeWeight(1);
      
      /* Acc Angle */
      if(accAngShow) {
        stroke(r_red_);
        angleGraph.drawLine(AccAng, -100, +100, scaleSliderAng.value());
      }
      
      /* Gyro Angle */
      if(gyroAngShow) {
        stroke(r_orange_);
        angleGraph.drawLine(GyroAng, -100, +100, scaleSliderAng.value());
      }
      
      /* Pitch */
      if(pitchAngShow) {
        stroke(yellow_);
        angleGraph.drawLine(PitchAng, -100, +100, scaleSliderAng.value());
      }
      
      /* Error */
      if(errorAngShow) {
        stroke(r_green_);
        angleGraph.drawLine(ErrorAng, -100, +100, scaleSliderAng.value());        
      }
      
      /* Setpoint */
      if(spAngShow){
        stroke(r_blue_);
        angleGraph.drawLine(SetAng, -100, +100, scaleSliderAng.value());
      }
      
      /* draw graph lines - PID graph */
      if(pValShow) {
        stroke(r_red_);
        pidGraph.drawLine(TotPid, -300, +300, scaleSliderPid.value());       
      }
      if(pTValShow) {
        stroke(r_orange_);
        pidGraph.drawLine(PtPid, -300, +300, scaleSliderPid.value());       
      }
      if(iTValShow) {
        stroke(yellow_);
        pidGraph.drawLine(ItPid, -300, +300, scaleSliderPid.value());       
      }
      if(dTValShow) {
        stroke(r_green_);
        pidGraph.drawLine(DtPid, -300, +300, scaleSliderPid.value());       
      }      
      if(lpValShow) { }
      if(rpValShow) { } 
      
      update_timer = time;     
    } 
  }
  
   /* Update status indicators */
   if(disabled){
     disabledToggle.setValue(true);
   } else {
     disabledToggle.setValue(false);
   }
 
   if(brake){
     brakeToggle.setValue(true);
   } else {
     brakeToggle.setValue(false);
   }

    /* Update comm port connection button */
    if(commSelected && baudSelected && !commConnected){
      commConnectButton.setColorBackground(green_);
    } else if(commSelected && baudSelected && commConnected){
      commConnectButton.setColorBackground(red_);
    } else {
      commConnectButton.setColorBackground(blue_);
    }    
    
    /* Update angle pause button */
    if(commConnected){
      if(anglePause){
        anglePauseButton.setColorBackground(red_);
      }
      else {
        anglePauseButton.setColorBackground(green_);
      }  
    }
    else {
      anglePauseButton.setColorBackground(blue_);  
    }
    
    /* Update pid pause button */
    if(commConnected){
      if(pidPause){
        pidPauseButton.setColorBackground(red_);
      }
      else {
        pidPauseButton.setColorBackground(green_);
      }  
    }
    else {
      pidPauseButton.setColorBackground(blue_);  
    }    
    
    /* Update write Button */
    if(commConnected){
      writeButton.setColorBackground(green_);   
    }
    else {
      writeButton.setColorBackground(red_);      
    }
    
    /* Disable PID/Gain sliders until we have received some initial values from the robot */
    if(serial_read){      
      pSlider.setLock(false);
      iSlider.setLock(false);
      dSlider.setLock(false);
      
      lftMotorSlider.setLock(false);
      rgtMotorSlider.setLock(false);
    } else {      
      pSlider.setLock(true);
      iSlider.setLock(true);
      dSlider.setLock(true);
      
      lftMotorSlider.setLock(true);
      rgtMotorSlider.setLock(true);      
    }
}

/* Generic event handling function */
public void controlEvent(ControlEvent theEvent) {  
  
  /* serial port list box */
  if(theEvent.isGroup() && theEvent.name().equals("commListbox")){
    commSelected = true;
    commPort = (int) theEvent.group().value();  
    
    statusTxtLbl.setValue(Serial.list()[commPort] + " selected");
  }    
  
  /* baud rate list box */
  if(theEvent.isGroup() && theEvent.name().equals("baudListbox")){
    baudSelected = true;
    baudRate = (int) theEvent.group().value();  
    
    statusTxtLbl.setValue(baudRate + " selected");    
  }     
}

/* commConnectButton: used to connect/disconnect serial comm port */
public void commConnectButton(int theValue) {
  if(commConnected && commSelected){
    boolean status = closeSerial();
    
    if(status){    
      commConnected = false;
      commConnectButton.setLabel("         Serial Connect");
    } else {
      statusTxtLbl.setValue("Error closing port");
    }
  }
  else {
     if(commSelected && baudSelected){
       boolean status = initSerial(commPort);
       
       if(status){
         commConnected = true;
         commConnectButton.setLabel("      Serial Disconnect");
         
         statusTxtLbl.setValue("    Connected!");
       } else {
         statusTxtLbl.setValue("Error connecting port");
       }
     }
     else {
       if(!commSelected && !baudSelected){
          statusTxtLbl.setValue("No port or baud selected!");         
       }
       else if(!commSelected){
          statusTxtLbl.setValue("No port selected!");         
       }
       else if(!baudSelected){
          statusTxtLbl.setValue("No baud selected!");         
       }
       else {
          statusTxtLbl.setValue("Unknown error!");                 
       }
     }   
  }
}

/* initialise the selected serial port */
boolean initSerial(int portVal) {
  if (portVal < commListMax) {
    String port = Serial.list()[portVal];
    
    try{
      serial = new Serial(this, port, baudRate);
    }
    catch(Exception e){
      println(e.getMessage());
      return false;
    }
    
    serial.bufferUntil('\n');  
    
    return true;
  } else {
    return false;
  }
}

/* close the currently open serial port */
boolean closeSerial(){
  if(commConnected){
    try {
      serial.clear();
      serial.stop();
      
      serial_read = false;
    }
    catch(Exception e){
      println(e.getMessage());
      return false;
    }
    
    return true;
  }
  else {
    return false;
  }
}

/*
 * Process serial events - format of incoming serial data is as follows:
 *
 * acc_angle, gyro_angle, gyro_rate, pitch, accel_x, accel_y, accel_z,
 * gyro_x, gyro_y, gyro_z, pid_value, pT, iT, dT, error, target, p, i, d,
 * motor_gain_left, motor_gain_right, disabled, brake
 *
 * Not all values are currently graphed by the application
 */                          
void serialEvent(Serial myPort) {
  
  /* Read string from the serial port */
  String   inString = myPort.readString();
  String[] inValues = split(inString.trim(), ',');
  
  /* Parse variables from incoming string */
  try {
    acc_angle  = (float) (parseInt(inValues[0]) / 10000.00);
    gyro_angle = (float) (parseInt(inValues[1]) / 10000.00);
    gyro_rate =  (float) (parseInt(inValues[2]) / 10000.00);
    pitch = (float) (parseInt(inValues[3]) / 10000.00);
    accel_x = (float) (parseInt(inValues[4]) / 10000.00);
    accel_y = (float) (parseInt(inValues[5]) / 10000.00);
    accel_z = (float) (parseInt(inValues[6]) / 10000.00);
    gyro_x = (float) (parseInt(inValues[7]) / 10000.00);
    gyro_y = (float) (parseInt(inValues[8]) / 10000.00);
    gyro_z = (float) (parseInt(inValues[9]) / 10000.00);
    pid_value = (float) (parseInt(inValues[10]) / 10000.00);
    pT = (float) (parseInt(inValues[11]) / 10000.00);
    iT = (float) (parseInt(inValues[12]) / 10000.00);
    dT = (float) (parseInt(inValues[13]) / 10000.00);
    error = (float) (parseInt(inValues[14]) / 10000.00);
    target = (float) (parseInt(inValues[15]) / 10000.00);
    
    /* On the first read, set the PID/Gain values */
    if(!serial_read){
      p = (float) (parseInt(inValues[16]) / 10000.00);
      i = (float) (parseInt(inValues[17]) / 10000.00);
      d = (float) (parseInt(inValues[18]) / 10000.00);
      motor_gain_left  = (float) (parseInt(inValues[19]) / 10000.00);
      motor_gain_right = (float) (parseInt(inValues[20]) / 10000.00);
      
      serial_read = true;
      
      pSlider.setValue(p);
      iSlider.setValue(i);
      dSlider.setValue(d);  
    
      lftMotorSlider.setValue(motor_gain_left);
      rgtMotorSlider.setValue(motor_gain_right);         
    }
    
    disabled = boolean(parseInt(inValues[21]));
    brake = boolean(parseInt(inValues[22]));
  }
  catch(Exception e){
      println("ERROR: " + e.getMessage());    
      if(!serial_read){
        return;
      }
  }
}

/* Send values to the robot */
void serialSend(){
  if(serial_read && commConnected){
    StringBuffer sb = new StringBuffer();
    
    /* build string to send to robot */
    sb.append(p + ",");
    sb.append(i + ",");
    sb.append(d + ",");
    sb.append(motor_gain_left + ",");
    sb.append(motor_gain_right + "\n");
    
    println(p + " : " + i + " : " + d);

    try {
      /* write out resulting string */
      serial.write(sb.toString());
    }
    catch(Exception e) {
       println(e.getMessage());
    }     
  }  
}

/* writeButton: write values to serial port */
public void writeButton(boolean theValue) {
  serialSend();
}

/* accAngToggle: toggle graph display of accelerometer angle */
public void accAngToggle(boolean theValue) {
  accAngShow ^= true;
}

/* gyroAngToggle: toggle graph display of gyroscope angle */
public void gyroAngToggle(boolean theValue) {
  gyroAngShow ^= true;
}

/* pitchAngToggle: toggle graph display of fused pitch angle */
public void pitchAngToggle(boolean theValue) {
  pitchAngShow ^= true;
}

/* errorAngToggle: toggle graph display of error angle */
public void errorAngToggle(boolean theValue) {
  errorAngShow ^= true;
}

/* spAngToggle: toggle graph display of setpoint angle */
public void spAngToggle(boolean theValue) {
  spAngShow ^= true;
}

/* pValToggle: toggle graph display of calculated PID value */
public void pValToggle(boolean theValue) {
  pValShow ^= true;
}

/* pTValToggle: toggle graph display of calculated PID pT value */
public void pTValToggle(boolean theValue) {
  pTValShow ^= true;
}

/* iTValToggle: toggle graph display of calculated PID iT value */
public void iTValToggle(boolean theValue) {
  iTValShow ^= true;
}

/* dTValToggle: toggle graph display of calculated PID dT value */
public void dTValToggle(boolean theValue) {
  dTValShow ^= true;
}

/* lpValToggle: toggle graph display of calculated left PID value */
public void lpValToggle(boolean theValue) {
  lpValShow ^= true;
}

/* rpValToggle: toggle graph display of calculated right PID value */
public void rpValToggle(boolean theValue) {
  rpValShow ^= true;
}

/* pSlider: adjusts the P gain of the PID controller */
public void pSlider(float theValue) {
  p = (float) theValue;
}

/* iSlider: adjusts the I gain of the PID controller */
public void iSlider(float theValue) {
  i = (float) theValue; 
}

/* dSlider: adjusts the D gain of the PID controller */
public void dSlider(float theValue) {
  d = (float) theValue;
}

/* lftMotorSlider: adjusts the gain of the left motor */
public void lftMotorSlider(float theValue) {
  motor_gain_left = (float) theValue;
}

/* rgtMotorSlider: adjusts the gain of the right motor */
public void rgtMotorSlider(float theValue) {
  motor_gain_right = (float) theValue;  
}

/* pPause: pause the output of the angle graph */
public void aPause(int theValue) {
  anglePause ^= true;
}

/* pPause: pause the output of the pid graph */
public void pPause(int theValue) {
  pidPause ^= true;
}

//********************************************************
//****************** Supporting Classes ******************
//********************************************************

class DataArray {
  float[] data;
  int maxSize, startIndex = 0, endIndex = 0, curSize;
  
  DataArray(int maxSize){
    this.maxSize = maxSize;
    data = new float[this.maxSize];
  }
  
  public void addVal(float val) {
    data[endIndex] = val;
    endIndex = (endIndex+1) % maxSize;
    if (curSize == maxSize) {
      startIndex = (startIndex+1) % maxSize;
    } else {
      curSize++;
    }
  }
  
  public float getVal(int index) {
    return data[(startIndex+index) % maxSize];
  }
  
  public int getCurSize(){
    return curSize;
  }
  
  public int getMaxSize() {
    return maxSize;
  }
  
  public float getMaxVal() {
    float res = 0.0f;
    for( int i=0; i < curSize - 1; i++) if ((data[i] > res) || (i==0)) res = data[i];
    return res;
  }
  
  public float getMinVal() {
    float res = 0.0f;
    for( int i=0; i < curSize-1; i++) if ((data[i] < res) || (i==0)) res = data[i];
    return res;
  }
  
  public float getRange() {
    return getMaxVal() - getMinVal();
  }
}

/* Generic graph class */
class Graph {
  float gWidth, gHeight, gLeft, gBottom, gRight, gTop;
  
  Graph(float x, float y, float w, float h) {
    gWidth     = w; 
    gHeight    = h;
    
    gLeft      = x; 
    gBottom    = y;
    
    gRight     = x + w;
    gTop       = y + h;
  }
  
  /* Draw graph outline box */
  public void drawGraphBox() {
    stroke(0, 0, 0);
    strokeWeight(1);
    
    fill(145);
    
    rectMode(CORNERS);
    rect(gLeft, gBottom, gRight, gTop);
  }
  
  /* Plot the graph data */
  public void drawLine(DataArray data, float minRange, float maxRange, float scaleVal) {
    float graphMultX = gWidth / data.getMaxSize();
    float graphMultY = gHeight / (maxRange - minRange);
    
    for( int i = 0; i < data.getCurSize() - 1; ++i) {
      float x0 = i * graphMultX + gLeft;
      float y0 = gTop - (((data.getVal(i) - (maxRange + minRange) / 2) * scaleVal + (maxRange - minRange) / 2) * graphMultY);
      float x1 = (i + 1) * graphMultX + gLeft;
      float y1 = gTop - (((data.getVal(i + 1) - (maxRange+minRange) / 2) * scaleVal + (maxRange-minRange) / 2) * graphMultY);
      line(x0, y0, x1, y1);
    }
  }
}
