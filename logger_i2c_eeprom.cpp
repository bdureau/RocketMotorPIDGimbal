#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);   
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
 
}



void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure

}

void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }
}



int logger_I2C_eeprom::readFlightList() {
  eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

int logger_I2C_eeprom::writeFastFlight(int eeaddress){
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}

int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret =  readFlightList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
      break;
    Serial1.print("Flight Nbr: ");
    Serial1.println(i);
    Serial1.print("Start: ");
    Serial1.println(_FlightConfig[i].flight_start);
    Serial1.print("End: ");
    Serial1.println(_FlightConfig[i].flight_stop);
  }
  return i;
}

void logger_I2C_eeprom::setFlightStartAddress(int flightNbr, long startAddress)
{
  _FlightConfig[flightNbr].flight_start = startAddress;
}

void logger_I2C_eeprom::setFlightEndAddress(int flightNbr, long endAddress)
{
  _FlightConfig[flightNbr].flight_stop = endAddress;
}

void logger_I2C_eeprom::setFlightTimeData( long difftime)
{
  _FlightData.diffTime = difftime;
}
void logger_I2C_eeprom::setFlightAltitudeData( long altitude)
{
  _FlightData.altitude = altitude;
}
void logger_I2C_eeprom::setFlightTemperatureData( long temperature){
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure){
  _FlightData.pressure = pressure;
}
//void logger_I2C_eeprom::setFlightRocketPos(char *w, char *x, char *y, char *z )
void logger_I2C_eeprom::setFlightRocketPos(long w, long x, long y, long z )
{
    /*_FlightData.w[0] = w[0];
    _FlightData.w[1] = w[1];
    _FlightData.x[0] = x[0];
    _FlightData.x[1] = x[1];
    _FlightData.y[0] = y[0];
    _FlightData.y[1] = y[1];
    _FlightData.z[0] = z[0];
    _FlightData.z[1] = z[1];*/
    _FlightData.w =w;
    _FlightData.x = x;
    _FlightData.y = y;
    _FlightData.z = z;
}
void logger_I2C_eeprom::getFlightRocketPos(long *pos) {
  /* pos[0] = _FlightData.w;
    pos[1] = _FlightData.x;
    pos[2] = _FlightData.y;
    pos[3] = _FlightData.z;*/
}
void logger_I2C_eeprom::setFlightCorrection( long OutputX, long OutputY)
{
  _FlightData.OutputX = OutputX;
  _FlightData.OutputY = OutputY;
}
void logger_I2C_eeprom::setAcceleration(long X,long Y,long Z) {
_FlightData.accelX = X;
_FlightData.accelY = Y;
_FlightData.accelZ = Z;
}
void logger_I2C_eeprom::getFlightCorrection(long *cor) {
  /*cor[0] = _FlightData.OutputX;
    cor[1] = _FlightData.OutputY;*/
}
long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_stop;
}

long logger_I2C_eeprom::getFlightTimeData()
{
  return _FlightData.diffTime;
}
long logger_I2C_eeprom::getFlightAltitudeData()
{
  return _FlightData.altitude;
}

void logger_I2C_eeprom::PrintFlight(int flightNbr)
{
  long startaddress;
  long endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);
  //flight_type = getFlightType(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;
    Serial1.println("StartFlight;" );
    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();
      Serial1.print("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ",");
      Serial1.print(_FlightData.temperature);
      Serial1.print(",");
      Serial1.print(_FlightData.pressure);
      Serial1.print(",");
      Serial1.print(_FlightData.w);
      Serial1.print(",");
      Serial1.print(_FlightData.x);
      Serial1.print(",");
      Serial1.print(_FlightData.y);
      Serial1.print(",");
      Serial1.print(_FlightData.z);
     //Serial1.println("," + _FlightData.w[0]+ _FlightData.w[1]); //","+ _FlightData.x + "," + _FlightData.y + "," + _FlightData.z);

    }
    Serial1.println("EndFlight;" );
  }
  else
    Serial1.println(F("No such flight\n"));
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{
  int startaddress;
  int endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();
      //long pos[4];
      //getFlightRocketPos(pos);
      Serial1.print("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ",");
      Serial1.print(_FlightData.temperature);
      Serial1.print(",");
      Serial1.print(_FlightData.pressure);
      Serial1.print(",");
      serialFloatPrint((float)(_FlightData.w)/1000);
      Serial1.print(",");
      serialFloatPrint((float)(_FlightData.x)/1000);
      Serial1.print(",");
      serialFloatPrint((float)(_FlightData.y)/1000);
      Serial1.print(",");
      serialFloatPrint((float)(_FlightData.z)/1000);
      Serial1.print(",");
      Serial1.print(_FlightData.OutputX);
      Serial1.print(",");
      Serial1.print(_FlightData.OutputY);
      Serial1.print(",");
      Serial1.print(_FlightData.accelX);
      Serial1.print(",");
      Serial1.print(_FlightData.accelY);
      Serial1.print(",");
      Serial1.print(_FlightData.accelZ);
      Serial1.println(";");
    }
  }
}
long logger_I2C_eeprom::getSizeOfFlightData()
{
  return sizeof(_FlightData);
}
boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;
  // Serial.println(lastFlight);
  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}
/*
 * 
 * getLastFlightNbr()
 * Parse the flight index end check if the flight_start address is > 0
 * return -1 if no flight have been recorded else return the flight number
 * 
 */
long logger_I2C_eeprom::getLastFlightEndAddress()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return _FlightConfig[i].flight_stop;
}
