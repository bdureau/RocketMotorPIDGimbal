#include "config.h"
ConfigStruct config;
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
   
  config.cksum=CheckSumConf(config);
  //config.cksum=0xBA; 
}

unsigned int CheckSumConf( ConfigStruct cnf)
 {
     int i;
     unsigned int chk=0;
    
     //for (i=0; i < (sizeof(cnf)-2); i++) 
     for (i=0; i < (sizeof(cnf)-sizeof(int)); i++) 
     chk += *((char*)&cnf + i);
    
     return chk;
 }

boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for( i=0; i< sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
  //if ( config.cksum != 0xBA ) {
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
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //Serial1.println(str);
    switch (i)
    {
    case 1:
      config.ax_offset =atoi(str);
      break;
    case 2:
      config.ay_offset =atoi(str);
      break;
    case 3:
      config.az_offset =atoi(str);
      break;
    case 4:
      config.gx_offset =atoi(str);
      break;
    case 5:
      config.gy_offset =atoi(str);
      break;
    case 6:
      config.gz_offset =atoi(str);
      break; 
    case 7:        
      config.KpX = atoi(str);
      break; 
    case 8:
      config.KiX = atoi(str);
      break; 
    case 9:
      config.KdX = atoi(str);
      break; 
    case 10:
      config.KpY = atoi(str);
      break; 
    case 11:
      config.KiY = atoi(str);
      break; 
    case 12:
      config.KdY = atoi(str);
      break; 
    case 13:
      config.ServoXMin =atoi(str);
      break; 
    case 14:
      config.ServoXMax =atoi(str);
      break; 
    case 15:
      config.ServoYMin =atoi(str);
      break; 
    case 16:
      config.ServoYMax =atoi(str);
      break; 
    case 17:  
      config.connectionSpeed = atoi(str);
      break; 
    case 18:  
      config.altimeterResolution = atoi(str);
      break; 
    case 19:  
      config.eepromSize =atoi(str);
      break; 
    case 20:  
      config.unit = atoi(str);
      break; 
    case 21:  
      config.endRecordAltitude=atoi(str);
      break; 
    case 22:  
      config.beepingFrequency=atoi(str);
      break;  
    case 23:
      config.liftOffDetect=atoi(str);  
      break;
    }
    i++;

  }

  //we have a partial config
  if (i<22)
    return false;
    
  //calculate checksum
  config.cksum = CheckSumConf(config);

  writeConfigStruc();
  return true;
}      

void writeConfigStruc()
{
    int i;
    for( i=0; i<sizeof(config); i++ ) {
      EEPROM.write(CONFIG_START+i, *((char*)&config + i));
    }
}
