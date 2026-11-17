/*
 * File:        config.h
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Firmware version, board constants and the configuration
 *              structure that is stored in the microcontroller (emulated)
 *              EEPROM and exchanged with the Android application.
 *
 * Changes:     - VOLT_DIVIDER now uses the real divider ratio (R1+R2)/R1.
 *              - Added tuning constants for the accelerometer liftoff
 *                detection, the IMU calibration and the serial command
 *                buffer.
 *
 * Warning:     The layout of ConfigStruct is stored in EEPROM and its
 *              fields are sent to the Android application. Do not reorder,
 *              add or remove fields.
 */
#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 3

#define BOARD_FIRMWARE "RocketMotorGimbal"
// Offset of the configuration structure in the internal EEPROM
#define CONFIG_START 32
extern const int pinSpeaker;
// Battery voltage below which the low battery alarm is triggered (Volts)
#define BAT_MIN_VOLTAGE 6.5

// Battery voltage divider (kOhm):
//   Battery + ---- R2 ----+---- R1 ---- GND
//                         |
//                        ADC (PB1)
// Vbat = Vadc * (R1 + R2) / R1
#define R1 4.7
#define R2 10
#define VOLT_DIVIDER ((R1 + R2) / R1)

// Accelerometer liftoff detection threshold, in g, on the rocket axis (Y).
// 1.83 g is the value of the former hard coded threshold (30000 raw counts
// at +/-2 g). It is converted to raw counts according to the configured
// accelerometer range.
#define LIFTOFF_ACCEL_THRESHOLD_G 1.83f

// Maximum number of passes of the IMU calibration before giving up.
// One pass takes roughly 0.6 s.
#define CALIBRATION_MAX_PASSES 100

// Size of the serial command buffer (longest accepted command - 1)
#define COMMAND_BUFFER_SIZE 1000

#include "Arduino.h"

// Uncomment to get additional debugging messages on the serial port
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG

// Used for writing in the microcontroller internal EEPROM
#include <EEPROM.h>

struct ConfigStruct {
  int ax_offset;            // MPU6050 accelerometer X offset
  int ay_offset;            // MPU6050 accelerometer Y offset
  int az_offset;            // MPU6050 accelerometer Z offset
  int gx_offset;            // MPU6050 gyroscope X offset
  int gy_offset;            // MPU6050 gyroscope Y offset
  int gz_offset;            // MPU6050 gyroscope Z offset
  int KpX;                  // PID X proportional gain
  int KiX;                  // PID X integral gain
  int KdX;                  // PID X derivative gain
  int KpY;                  // PID Y proportional gain
  int KiY;                  // PID Y integral gain
  int KdY;                  // PID Y derivative gain
  int ServoXMin;            // servo X minimum angle
  int ServoXMax;            // servo X maximum angle
  int ServoYMin;            // servo Y minimum angle
  int ServoYMax;            // servo Y maximum angle
  int connectionSpeed;      // serial speed
  int altimeterResolution;  // BMP085 oversampling mode (0 to 3)
  int eepromSize;           // external EEPROM size
  int unit;                 // 0 = meters, 1 = feet
  int endRecordAltitude;    // altitude at which the recording ends
  int beepingFrequency;     // buzzer frequency
  int liftOffDetect;        // 0 = barometer, 1 = accelerometer
  int gyroRange;            // MPU6050 gyro range (not applied, see initialize())
  int acceleroRange;        // MPU6050 accelerometer range (0 = 2g ... 3 = 16g)
  int recordingTimeout;     // maximum recording duration in seconds
  int batteryType;          // 0 = unknown, 1 = 2S, 2 = 9V, 3 = 3S
  int telemetryType;        // telemetry module type (slows down the dumps)
  int cksum;                // checksum of all the fields above
};
extern ConfigStruct config;
extern void defaultConfig();
extern boolean readAltiConfig();
extern void writeConfigStruc();
extern unsigned int CheckSumConf( ConfigStruct );
extern bool writeAltiConfigV2( char *p );

extern void longBeep();
extern void shortBeep();
extern void beepAltiVersion (int majorNbr, int minorNbr);
extern unsigned int msgChk( char * buffer, long length );

#endif
