#include "config.h"
ConfigStruct config;
const int pinSpeaker = PA0;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  config.ax_offset =0;
  config.ay_offset =0;
  config.az_offset =0;
  config.gx_offset =0;
  config.gy_offset =0;
  config.gz_offset =0;
  config.KpX=0;
  config.KiX=0;
  config.KdX=0;
  config.KpY=0;
  config.KiY=0;
  config.KdY=0;
  config.ServoXMin=0;
  config.ServoXMax=0;
  config.ServoYMin=0;
  config.ServoYMax=0;
  config.connectionSpeed=38400;
  config.altimeterResolution=0;
  config.eepromSize=512;
  config.unit=0;
  config.endRecordAltitude=3;
  config.beepingFrequency=440;
  config.liftOffDetect=0;
  config.gyroRange=0;
  config.acceleroRange=0; 
  config.cksum=CheckSumConf(config);
  //config.cksum=0xBA; 
}



boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for( i=0; i< sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
  
    return false;
  }

  return true;

}

/*
* write the config received by the console
*
*/
bool  writeAltiConfig( char *p ) {
//Serial1.println(p);
  char *str;

  int i=0;
  int strChk=0;
  char msg[100]="";
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //Serial1.println(str);
    switch (i)
    {
    case 1:
      config.ax_offset =atoi(str);
      strcat(msg, str);
      break;
    case 2:
      config.ay_offset =atoi(str);
      strcat(msg, str);
      break;
    case 3:
      config.az_offset =atoi(str);
      strcat(msg, str);
      break;
    case 4:
      config.gx_offset =atoi(str);
      strcat(msg, str);
      break;
    case 5:
      config.gy_offset =atoi(str);
     strcat(msg, str);
      break;
    case 6:
      config.gz_offset =atoi(str);
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
      config.ServoXMin =atoi(str);
      strcat(msg, str);
      break; 
    case 14:
      config.ServoXMax =atoi(str);
      strcat(msg, str);
      break; 
    case 15:
      config.ServoYMin =atoi(str);
      strcat(msg, str);
      break; 
    case 16:
      config.ServoYMax =atoi(str);
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
      config.eepromSize =atoi(str);
      strcat(msg, str);
      break; 
    case 20:  
      config.unit = atoi(str);
      strcat(msg, str);
      break; 
    case 21:  
      config.endRecordAltitude=atoi(str);
      strcat(msg, str);
      break; 
    case 22:  
      config.beepingFrequency=atoi(str);
      strcat(msg, str);
      break;  
    case 23:
      config.liftOffDetect=atoi(str);
      strcat(msg, str); 
      break; 
    case 24:  
      config.gyroRange=atoi(str);
      strcat(msg, str);
      break; 
    case 25:  
      config.acceleroRange=atoi(str);
      strcat(msg, str);
      break;
    case 26:
      //our checksum
      strChk= atoi(str);
      //strcat(msg, str);
      break; 
    }
    i++;

  }
//msg[i-1]='\0';
//Serial1.println(msg);
//Serial1.println(strChk);
  //we have a partial config
  if (i<25)
    return false;
  if(msgChk(msg, sizeof(msg)) != strChk)
     return false;
  //calculate checksum
  config.cksum = CheckSumConf(config);
 
  writeConfigStruc();
  return true;
}      

unsigned int msgChk( char * buffer, long length ) {

     long index;
     unsigned int checksum;

     for( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
     return (unsigned int) ( checksum % 256 );

}

unsigned int CheckSumConf( ConfigStruct cnf)
 {
     int i;
     unsigned int chk=0;
    
     for (i=0; i < (sizeof(cnf)-sizeof(int)); i++) 
     chk += *((char*)&cnf + i);
    
     return chk;
 }
void writeConfigStruc()
{
    int i;
    for( i=0; i<sizeof(config); i++ ) {
      EEPROM.write(CONFIG_START+i, *((char*)&config + i));
    }
}
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
