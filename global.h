#ifndef _GLOBAL_H
#define _GLOBAL_H
#define INTERRUPT_PIN PB12
#define LED_PIN PC13 //pin 13 for the arduino Uno and PC13 for the stm32 
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>
#include <Servo.h> //servo library
#include <PWMServo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h> // Gyroscope and axcelerometer libraries
#include <PID_v1.h> // Arduino PID library
#include <Wire.h>
//#include <Adafruit_BMP085.h>
#include <BMP085_stm32.h>

BMP085 bmp;
bool blinkState = true;
bool telemetryEnable = false;
bool mainLoopEnable = true;

Servo ServoX;   // X axis Servo
Servo ServoY;   // Y axis Servo
//PWMServo ServoX;
//PWMServo ServoY;
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;
float correct;


// Create  MPU object
// class default I2C address is 0x68; specific I2C addresses may be passed as a parameter here
// for exemple MPU6050 mpu(0x69);
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
Quaternion hq;          // quaternion conjugate
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   2//1     // X defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  0     // Y defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW  1// 2     // Z defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// PID stuff
//Define Variables we'll be connecting to
double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

// Those initial tuning parameters need to be tuned
// this is a bit like when you tune your copter appart from the fact that the rocket motor last only few seconds
// please help !!!!!!!
//double KpX = 2, KiX = 5, KdX = 1;
double KpX = 3.55, KiX = 0.005, KdX = 2.05;
//Specify the links and initial tuning parameters
//double KpY = 2, KiY = 5, KdY = 1;
double KpY = 3.55, KiY = 0.005, KdY = 2.05;
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);


//calibration stuff
//Change those 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 200;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
long last_telem_time=0;
#endif
