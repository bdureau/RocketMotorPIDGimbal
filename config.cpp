/*
 * File:        config.cpp
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Description: Default configuration, read/write of the configuration in
 *              the microcontroller (emulated) EEPROM, configuration update
 *              received from the Android application, message checksum and
 *              buzzer helpers.
 *
 * Changes:     - writeConfigStruc() now uses the buffered EEPROM API of the
 *                STM32 core: the flash page is erased and programmed once
 *                instead of once per byte (faster and much less flash wear).
 */
#include "config.h"
ConfigStruct config;
const int pinSpeaker = PA0;
//================================================================
// Read and write the configuration in the microcontroller EEPROM
//================================================================

/*
   defaultConfig()
   Load the factory default values in the configuration structure.
*/
void defaultConfig()
{
  config.ax_offset = 0;
  config.ay_offset = 0;
  config.az_offset = 0;
  config.gx_offset = 0;
  config.gy_offset = 0;
  config.gz_offset = 0;
  config.KpX = 0;
  config.KiX = 0;
  config.KdX = 0;
  config.KpY = 0;
  config.KiY = 0;
  config.KdY = 0;
  config.ServoXMin = 0;
  config.ServoXMax = 0;
  config.ServoYMin = 0;
  config.ServoYMax = 0;
  config.connectionSpeed = 38400;
  config.altimeterResolution = 0;
  config.eepromSize = 512;
  config.unit = 0;
  config.endRecordAltitude = 3;
  config.beepingFrequency = 440;
  config.liftOffDetect = 0;
  config.gyroRange = 0;
  config.acceleroRange = 0;
  config.recordingTimeout = 120;
  config.batteryType = 0; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  config.telemetryType = 0;
  config.cksum = CheckSumConf(config);
}



/*
   readAltiConfig()
   Read the configuration from the EEPROM.
   Return false if the checksum is invalid (configuration never written
   or corrupted).
*/
boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {

    return false;
  }

  return true;

}

/*
  Former version of the configuration writer (whole configuration in one
  message). Kept for reference only, replaced by writeAltiConfigV2().
*/
/*bool  writeAltiConfig( char *p ) {
  //Serial1.println(p);
  char *str;

  int i = 0;
  int strChk = 0;
  char msg[100] = "";
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //Serial1.println(str);
    switch (i)
    {
      case 1:
        config.ax_offset = atoi(str);
        strcat(msg, str);
        break;
      case 2:
        config.ay_offset = atoi(str);
        strcat(msg, str);
        break;
      case 3:
        config.az_offset = atoi(str);
        strcat(msg, str);
        break;
      case 4:
        config.gx_offset = atoi(str);
        strcat(msg, str);
        break;
      case 5:
        config.gy_offset = atoi(str);
        strcat(msg, str);
        break;
      case 6:
        config.gz_offset = atoi(str);
        strcat(msg, str);
        break;
      case 7:
        config.KpX = atoi(str);
        strcat(msg, str);
        break;
      case 8:
        config.KiX = atoi(str);
        strcat(msg, str);
        break;
      case 9:
        config.KdX = atoi(str);
        strcat(msg, str);
        break;
      case 10:
        config.KpY = atoi(str);
        strcat(msg, str);
        break;
      case 11:
        config.KiY = atoi(str);
        strcat(msg, str);
        break;
      case 12:
        config.KdY = atoi(str);
        strcat(msg, str);
        break;
      case 13:
        config.ServoXMin = atoi(str);
        strcat(msg, str);
        break;
      case 14:
        config.ServoXMax = atoi(str);
        strcat(msg, str);
        break;
      case 15:
        config.ServoYMin = atoi(str);
        strcat(msg, str);
        break;
      case 16:
        config.ServoYMax = atoi(str);
        strcat(msg, str);
        break;
      case 17:
        config.connectionSpeed = atoi(str);
        strcat(msg, str);
        break;
      case 18:
        config.altimeterResolution = atoi(str);
        strcat(msg, str);
        break;
      case 19:
        config.eepromSize = atoi(str);
        strcat(msg, str);
        break;
      case 20:
        config.unit = atoi(str);
        strcat(msg, str);
        break;
      case 21:
        config.endRecordAltitude = atoi(str);
        strcat(msg, str);
        break;
      case 22:
        config.beepingFrequency = atoi(str);
        strcat(msg, str);
        break;
      case 23:
        config.liftOffDetect = atoi(str);
        strcat(msg, str);
        break;
      case 24:
        config.gyroRange = atoi(str);
        strcat(msg, str);
        break;
      case 25:
        config.acceleroRange = atoi(str);
        strcat(msg, str);
        break;
      case 26:
        config.recordingTimeout = atoi(str);
        strcat(msg, str);
        break;
      case 27:
        config.batteryType = atoi(str);
        strcat(msg, str);
        break;   
      case 28:
        //our checksum
        strChk = atoi(str);
        //strcat(msg, str);
        break;
    }
    i++;

  }

  //we have a partial config
  if (i < 27)
    return false;
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;
  //calculate checksum
  config.cksum = CheckSumConf(config);

  writeConfigStruc();
  return true;
}*/

/*
   writeAltiConfigV2()
   Update one configuration parameter received from the Android application.
   Message format: p,<parameter number>,<value>,<checksum>
   The configuration is only updated in RAM, the 'q' command saves it.
   Return false if the message is incomplete or the checksum is invalid.
*/
bool writeAltiConfigV2( char *p ) {

  char *str;
  int i = 0;
  int command =0;
  long commandVal =0;
  int strChk = 0;
  char msg[100] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    if (i == 1) {
      command = atoi(str);
      strcat(msg, str);
    }
    if (i == 2) {
      commandVal =  atol(str);
      strcat(msg, str);
    }
    if (i == 3) {
      strChk  =  atoi(str);  
    }
    i++;

  }
  //we have a partial config
  if (i < 4)
    return false;
  //is the checksum invalid?
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;  
    
  switch (command)
    {
      case 1:
        config.ax_offset = (int) commandVal;
        break;
      case 2:
        config.ay_offset = (int) commandVal;
        break;
      case 3:
        config.az_offset = (int) commandVal;
        break;
      case 4:
        config.gx_offset = (int) commandVal;
        break;
      case 5:
        config.gy_offset = (int) commandVal;
        break;
      case 6:
        config.gz_offset = (int) commandVal;
        break;
      case 7:
        config.KpX = (int) commandVal;
        break;
      case 8:
        config.KiX = (int) commandVal;
        break;
      case 9:
        config.KdX = (int) commandVal;
        break;
      case 10:
        config.KpY = (int) commandVal;
        break;
      case 11:
        config.KiY = (int) commandVal;
        break;
      case 12:
        config.KdY = (int) commandVal;
        break;
      case 13:
        config.ServoXMin = (int) commandVal;
        break;
      case 14:
        config.ServoXMax = (int) commandVal;
        break;
      case 15:
        config.ServoYMin = (int) commandVal;
        break;
      case 16:
        config.ServoYMax = (int) commandVal;
        break;
      case 17:
        config.connectionSpeed = commandVal;
        break;
      case 18:
        config.altimeterResolution = (int) commandVal;
        break;
      case 19:
        config.eepromSize = (int) commandVal;
        break;
      case 20:
        config.unit = (int) commandVal;
        break;
      case 21:
        config.endRecordAltitude = (int) commandVal;
        break;
      case 22:
        config.beepingFrequency = (int) commandVal;
        break;
      case 23:
        config.liftOffDetect = (int) commandVal;
        break;
      case 24:
        config.gyroRange = (int) commandVal;
        break;
      case 25:
        config.acceleroRange = (int) commandVal;
        break;
      case 26:
        config.recordingTimeout = (int) commandVal;
        break;
      case 27:
        config.batteryType = (int) commandVal;
        break; 
      case 28:  
        config.telemetryType = (int) commandVal;     
        break;      
    }

  // add checksum
  config.cksum = CheckSumConf(config);

  return true;
}
/*
   msgChk()
   Checksum of a message: sum of all the bytes modulo 256.
   Trailing null bytes do not change the result.
*/
unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}

/*
   CheckSumConf()
   Checksum of the configuration structure (all the fields except cksum).
*/
unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}

/*
   writeConfigStruc()
   Save the configuration structure in the microcontroller EEPROM.

   On STM32 the EEPROM is emulated in flash and every EEPROM.write() call
   erases and re-programs the whole flash page. The buffered API is used
   instead: the page is copied to RAM, modified, then flashed only once.
*/
void writeConfigStruc()
{
  unsigned int i;
#if defined(ARDUINO_ARCH_STM32)
  // load the current flash page content so that the other bytes are kept
  eeprom_buffer_fill();
  for ( i = 0; i < sizeof(config); i++ ) {
    eeprom_buffered_write_byte(CONFIG_START + i, *((uint8_t*)&config + i));
  }
  // single erase + program of the flash page
  eeprom_buffer_flush();
#else
  // other cores: fall back to the byte by byte write
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
#endif
}
/*
   Buzzer helpers
*/
void longBeep()
{
  //if (NoBeep == false)
  //{
  tone(pinSpeaker, 440, 1000);
  delay(1500);
  noTone(pinSpeaker);
  //}
}
void shortBeep()
{
  // if (NoBeep == false)
  //{
  tone(pinSpeaker, 440, 25);
  delay(300);
  noTone(pinSpeaker);
  //}
}
/*
   beepAltiVersion()
   Beep the firmware version: one long beep per major number then one
   short beep per minor number.
*/
void beepAltiVersion (int majorNbr, int minorNbr)
{
  int i;
  for (i = 0; i < majorNbr; i++)
  {
    longBeep();
  }
  for (i = 0; i < minorNbr; i++)
  {
    shortBeep();
  }
}
