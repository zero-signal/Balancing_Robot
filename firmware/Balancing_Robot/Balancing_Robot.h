#ifndef _BERTY_H_
#define _BERTY_H_

#undef int
#include <stdio.h>
#include <string.h>

//#############################################################################
// Serial variables
#define SERIAL_WRITE_BYTES 768
#define SERIAL_READ_BYTES  35

uint8_t  serial_out_update = 100;
uint8_t  serial_in_update  = 100;
uint32_t  serial_out_timer, serial_in_timer;

char serial_out_buf[SERIAL_WRITE_BYTES];
char serial_in_buf[SERIAL_READ_BYTES];

#define FORMAT_STR_LEN    105
char fmt[FORMAT_STR_LEN];

//#############################################################################
// Angle variables

const double max_angle    =  45.00;                   // our maximum 'tilt' angle
const double target_angle =  1.00;                    // our target angle, i.e. upgright ;-)

const int zG[3] = { -20, 15, -23 };                   // zeroG values for ADXL345 acc
double acc_xG, acc_yG, acc_zG                  = 0;   // computed G values from acc
double acc_angle, gyro_angle, gyro_rate, pitch = 0;   // angle calculation variables and results

Kalman kalman;

boolean disabled = true;
boolean brake    = true;

//#############################################################################
// PID variables
double last_angle;
double last_error;
double integrated_error;

double error;
double pT, iT, dT;
double pid_value, pid_left, pid_right;

// actual PID values, should be configurable somehow...
double p = 17.0;   //3.5
double i = 4.0;   //4.0
double d = 5.0;   //3.0

//#############################################################################
// Timer variables
uint32_t kalman_timer;
uint32_t nunchuck_timer;

#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long last_loop_time = STD_LOOP_TIME;
unsigned long loop_start_time;

//#############################################################################
// GY-80 IMU variables

/* ------- ADXL345 Accelerometer ------- */
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32

#define ADXL345_NO_ERROR  0   // initial state
#define ADXL345_READ_ERROR 1  // problem reading accel
#define ADXL345_BAD_ARG  2    // bad method argument

#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

#define ADXL345_TWI_ADDR       ((byte)0x53)
#define ADXL345_TELEGRAM_LEN   ((byte)6)

struct imu_acc_data_s { 
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
};
imu_acc_data_s  *g_imu_acc_curr = new imu_acc_data_s();
static uint8_t   g_imu_acc_buf[ADXL345_TELEGRAM_LEN];
static uint32_t  g_imu_acc_seq = 0;

/* ------- L3G4200D Gyroscope ------- */
#define L3G4200D_CTRL_REG1  0x20
#define L3G4200D_CTRL_REG2  0x21
#define L3G4200D_CTRL_REG3  0x22
#define L3G4200D_CTRL_REG4  0x23
#define L3G4200D_CTRL_REG5  0x24
#define L3G4200D_CTRL_REG6  0x25

#define L3G4299D_OUT_X_L    0x28
#define L3G4299D_OUT_X_H    0x29
#define L3G4299D_OUT_Y_L    0x2A
#define L3G4299D_OUT_Y_H    0x2B
#define L3G4299D_OUT_Z_L    0x2C
#define L3G4299D_OUT_Z_H    0x2D

#define L3G4200D_TWI_ADDR       ((byte)0x69)
#define L3G4200D_TELEGRAM_LEN   ((byte)6)

#define L3G4200D_NUM_SAMPLES     50

#define L3G4200D_DPS_PER_DIGIT   0.00875

struct imu_gyro_data_s { 
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
};
imu_gyro_data_s  *g_imu_gyro_curr = new imu_gyro_data_s();
static uint8_t   g_imu_gyro_buf[L3G4200D_TELEGRAM_LEN];
static uint32_t  g_imu_gyro_seq = 0;

double gyro_zero;

//#############################################################################
// Motor variables
#define LEFT_MOTOR       0
#define RIGHT_MOTOR      1

// left motor
#define LEFT_MOTOR_DIR   12
#define LEFT_MOTOR_PWM   3
#define LEFT_MOTOR_BRK   9
#define LEFT_MOTOR_CNT   A0

// right motor
#define RIGHT_MOTOR_DIR  13
#define RIGHT_MOTOR_PWM  11
#define RIGHT_MOTOR_BRK  8
#define RIGHT_MOTOR_CNT  A1

//generic motor variables
#define LEFT_MOTOR_FORWARD    LOW
#define LEFT_MOTOR_BACKWARD   HIGH

#define RIGHT_MOTOR_FORWARD   HIGH
#define RIGHT_MOTOR_BACKWARD  LOW

//motor characteristics are not always equal...
double motor_gain_left  = 0.85;
double motor_gain_right = 1.0;

#endif

//end of file
