#ifndef _LOGGER_I2C_EEPROM_H
#define _LOGGER_I2C_EEPROM_H

#include <Wire.h>


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "Wstring.h"
#include "Wiring.h"
#endif



struct FlightDataStruct {
  long diffTime;
  long altitude;
  float w;
  float x;
  float y;
  float z;
  double OutputX;
  double OutputY;
};

struct FlightConfigStruct {
  long flight_start;    
  long flight_stop; 
  //long flight_type;
};

#define LOGGER_I2C_EEPROM_VERSION "1.0.0"

// The DEFAULT page size. This is overriden if you use the second constructor.
// I2C_EEPROM_PAGESIZE must be multiple of 2 e.g. 16, 32 or 64
// 24LC256 -> 64 bytes
#define LOGGER_I2C_EEPROM_PAGESIZE 64
#define FLIGHT_LIST_START 0
#define FLIGHT_DATA_START 200
class logger_I2C_eeprom
{
public:
    /**
     * Initializes the EEPROM with a default pagesize of I2C_EEPROM_PAGESIZE.
     */
    logger_I2C_eeprom(uint8_t deviceAddress);
    logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize);
    uint8_t _deviceAddress;
    void begin();
    void clearFlightList();
    void write_byte( unsigned int eeaddress, uint8_t data );
    uint8_t read_byte(  unsigned int eeaddress );
    int readFlight(int eeaddress);
    int writeFlight(int eeaddress);
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
    void setFlightCorrection( double OutputX, double OutputY);
    void getFlightCorrection(float *cor);
    void setFlightRocketPos(float pos[4]);
    void getFlightRocketPos(float *pos);
    long getFlightStart(int flightNbr);
    long getFlightStop(int flightNbr);
    //long getFlightType(int flightNbr);
    void PrintFlight(int flightNbr);
    void printFlightData(int flightNbr);
    boolean CanRecord();
    
private:
    
    FlightConfigStruct _FlightConfig[25];
    FlightDataStruct _FlightData;
};

#endif
// END OF FILE
