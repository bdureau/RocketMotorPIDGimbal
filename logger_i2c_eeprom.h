#ifndef _LOGGER_I2C_EEPROM_H
#define _LOGGER_I2C_EEPROM_H

#include <Wire.h>

#include "utils.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "Wstring.h"
#include "Wiring.h"
#endif
// TWI buffer needs max 2 bytes for eeprom address
// 1 byte for eeprom register address is available in txbuffer
#define I2C_TWIBUFFERSIZE  30

// to break blocking read/write after n millis()
#define I2C_EEPROM_TIMEOUT  1000


struct FlightDataStruct {
  long diffTime;
  long altitude;
  long temperature;
  long pressure;
 /* char w[2];
  char x[2];
  char y[2];
  char z[2];*/
  long w;
  long x;
  long y;
  long z;
  long OutputX;
  long OutputY;
  long accelX;
  long accelY;
  long accelZ;
};

struct FlightConfigStruct {
  long flight_start;    
  long flight_stop; 
};

#define LOGGER_I2C_EEPROM_VERSION "1.0.0"


#define FLIGHT_LIST_START 0
#define FLIGHT_DATA_START 200
class logger_I2C_eeprom
{
public:
    /**
     * Initializes the logger.
     */
    logger_I2C_eeprom(uint8_t deviceAddress);
    
    void begin();
    void clearFlightList();
    int readFlight(int eeaddress);
    int readFlightList();
    int writeFlightList();
    int getLastFlightNbr();
    int printFlightList();
    void setFlightStartAddress(int flightNbr, long startAddress);
    void setFlightEndAddress(int flightNbr, long endAddress);
    void setFlightTimeData( long difftime);
    long getFlightTimeData();
    void setFlightAltitudeData( long altitude);
    long getFlightAltitudeData();
    void setFlightPressureData( long pressure);
    void setFlightTemperatureData( long temperature);
    void setFlightCorrection( long OutputX, long OutputY);
    void setAcceleration(long X,long Y,long Z);
    void getFlightCorrection(long *cor);
    //void setFlightRocketPos(char *w, char *x, char *y, char *z);
    void setFlightRocketPos(long w, long x, long y, long z );
    void getFlightRocketPos(long *pos);
    long getFlightStart(int flightNbr);
    long getFlightStop(int flightNbr);
    void PrintFlight(int flightNbr);
    void printFlightData(int flightNbr);
    boolean CanRecord();
    int writeFastFlight(int eeaddress);
    long getSizeOfFlightData();
    long getLastFlightEndAddress();
    
private:
    
    FlightConfigStruct _FlightConfig[25];
    FlightDataStruct _FlightData;
    uint8_t _deviceAddress;
};

#endif
// END OF FILE
