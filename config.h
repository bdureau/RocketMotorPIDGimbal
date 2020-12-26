#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 1
#define CONFIG_START 32
extern const int pinSpeaker;
#define BAT_MIN_VOLTAGE 7.0
//Voltage divider
#define R1 4.7
#define R2 10

#define VOLT_DIVIDER 10*(R1/(R1+R2))
#include "Arduino.h"

// If you want to have additionnal debugging uncomment it
//#define SERIAL_DEBUG
#undef SERIAL_DEBUG

//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

struct ConfigStruct {
  int ax_offset;
  int ay_offset;
  int az_offset;
  int gx_offset;
  int gy_offset;
  int gz_offset;
  int KpX;
  int KiX;
  int KdX;
  int KpY;
  int KiY;
  int KdY;
  int ServoXMin;
  int ServoXMax;
  int ServoYMin;
  int ServoYMax;
  int connectionSpeed;
  int altimeterResolution;
  int eepromSize;
  int unit;
  int endRecordAltitude;
  int beepingFrequency;
  int liftOffDetect;
  int gyroRange;
  int acceleroRange;
  int cksum;
};
extern ConfigStruct config;
extern void defaultConfig();
extern boolean readAltiConfig();
extern void writeConfigStruc();
extern unsigned int CheckSumConf( ConfigStruct );
extern bool writeAltiConfig( char *p );

extern void longBeep();
extern void shortBeep();
extern void beepAltiVersion (int majorNbr, int minorNbr);
extern unsigned int msgChk( char * buffer, long length );

#endif
