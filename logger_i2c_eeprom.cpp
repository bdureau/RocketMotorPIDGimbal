#include "logger_i2c_eeprom.h"

logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{
  _deviceAddress = deviceAddress;
  logger_I2C_eeprom(deviceAddress, LOGGER_I2C_EEPROM_PAGESIZE);
}

logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress, const unsigned int deviceSize)
{
  _deviceAddress = deviceAddress;
  _pageSize = 64;//determineSize();

  // Chips 16Kbit (2048 Bytes) or smaller only have one-word addresses.
  // Also try to guess page size from device size (going by Microchip 24LCXX datasheets here).
  /*if (deviceSize <= 256)
    {
      this->_isAddressSizeTwoWords = false;
      this->_pageSize = 8;
    }
    else if (deviceSize <= 256 * 8)
    {
      this->_isAddressSizeTwoWords = false;
      this->_pageSize = 16;
    }
    else
    {
      this->_isAddressSizeTwoWords = true;
      this->_pageSize = 32;
    }*/
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



void logger_I2C_eeprom::write_byte( unsigned int eeaddress, uint8_t data ) {
  int rdata = data;
  int writeDelay = 10;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(writeDelay);
}


uint8_t logger_I2C_eeprom::read_byte(  unsigned int eeaddress ) {
  uint8_t rdata = 0xFF;
  int readDelay = 5;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

uint8_t logger_I2C_eeprom::_ReadBlock(  unsigned int eeaddress, uint8_t* buffer, const uint8_t length ) {
  //uint8_t rdata = 0xFF;
  int readDelay = 5;
  Wire.beginTransmission(_deviceAddress);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  int rv = Wire.endTransmission();
  if (rv != 0) return 0;  // error
  Wire.requestFrom((uint8_t)_deviceAddress, (uint8_t)length);
  uint8_t cnt = 0;
  uint32_t before = millis();
  while ((cnt < length) && ((millis() - before) < 1000))
  {
    if (Wire.available()) buffer[cnt++] = Wire.read();
  }
  return cnt;
}

int logger_I2C_eeprom::readFlightList() {

  int i;
  for ( i = 0; i < sizeof(_FlightConfig); i++ ) {
    // Serial.println(read_byte( FLIGHT_LIST_START+i ));
    *((char*)&_FlightConfig + i) = read_byte( FLIGHT_LIST_START + i );
  }
  return FLIGHT_LIST_START + i ;
}

int logger_I2C_eeprom::readFlight(int eeaddress) {

  int i;
  for ( i = 0; i < sizeof(_FlightData); i++ ) {
    *((char*)&_FlightData + i) = read_byte( eeaddress + i );
  }
  return eeaddress + i;
}

int logger_I2C_eeprom::writeFlightList()
{
  int i;
  for ( i = 0; i < sizeof(_FlightConfig); i++ ) {
    write_byte( FLIGHT_LIST_START + i, *((char*)&_FlightConfig + i) );
  }
  return FLIGHT_LIST_START + i;
}

int logger_I2C_eeprom::writeFlight(int eeaddress)
{
  int i;
  for ( i = 0; i < sizeof(_FlightData); i++ ) {
    write_byte( eeaddress + i, *((char*)&_FlightData + i) );
  }
  return eeaddress + i;
}


/*int  logger_I2C_eeprom::writeFastFlight(uint16_t eeaddress) {
  // Have to handle write page wrapping,
  // 24lc512 has 128 byte
  // 24lc64 has 32 byte

  //const uint16_t len;
  const uint8_t pageSize = _pageSize;
  uint16_t bk = sizeof(_FlightData);
  bool abort = false;
  uint8_t i;
  uint16_t j = 0;
  uint32_t timeout;
  uint16_t mask = pageSize - 1;
  while ((bk > 0) && !abort) {
    i = I2C_TWIBUFFERSIZE; // maximum data bytes that Wire.h can send in one transaction
    if (i > bk) i = bk; // data block is bigger than Wire.h can handle in one transaction
    if (((eeaddress) & ~mask) != ((((eeaddress) + i) - 1) & ~mask)) { // over page! block would wrap around page
      i = (((eeaddress) | mask) - (eeaddress)) + 1; // shrink the block until it stops at the end of the current page

    }
    //wait for the EEPROM device to complete a prior write, or 10ms
    timeout = millis();
    bool ready = false;
    while (!ready && (millis() - timeout < 10)) {
      Wire.beginTransmission(_deviceAddress);
      ready = (Wire.endTransmission(true) == 0); // wait for device to become ready!
    }
    if (!ready) { // chip either does not exist, is hung, or died
      abort = true;

      break;
    }

    // start sending this current block
    Wire.beginTransmission(_deviceAddress);
    Wire.write((uint8_t)highByte(eeaddress));
    Wire.write((uint8_t)lowByte(eeaddress));

    bk = bk - i;
    eeaddress = (eeaddress) + i;

    while (i > 0) {
      Wire.write(*((char*)&_FlightData + (j++)));
      i--;
    }

    uint8_t err = Wire.endTransmission();
    //delay(10);
    if(err!=0){
 Serial1.print(F("write Failure="));
 Serial1.println(err,DEC);
 //abort = true;

 }
  }

  return eeaddress;
}*/

int logger_I2C_eeprom::writeFastFlight(int eeaddress){//(uint16_t addr, const T& value){
      char* p = ((char*)&_FlightData); //(const uint8_t*)(const void*)&value;               
      int BLOCKSIZE =16;
      Wire.beginTransmission(_deviceAddress);
      Wire.write((int)(eeaddress >> 8));        // MSB
      Wire.write((int)(eeaddress & 0xFF));      // LSB

      //in the loop: counts the bytes we may send before 
      //our block becomes full
      //but initialise it to the number of bytes up to the
      //next 16-byte aligned address
      uint8_t blockBytes = (eeaddress/BLOCKSIZE + 1)*BLOCKSIZE - eeaddress;       
      int i;
      //int a=0;
      //Serial1.println(sizeof(_FlightData));
      for (i = 0; i < sizeof(_FlightData); i++){
            if (blockBytes == 0){
                  //block is full;
                  Wire.endTransmission(); //dispatch the buffer
                  delay(10);
                  //restart new block
                  eeaddress = (eeaddress/BLOCKSIZE + 1)*BLOCKSIZE;
                  blockBytes = BLOCKSIZE;
                  Wire.beginTransmission(_deviceAddress);
                  Wire.write((int)(eeaddress >> 8));  // MSB
                  Wire.write((int)(eeaddress & 0xFF));// LSB
            }
            //Wire.write(*p++); //dispatch the data byte
            //a++;
          Wire.write(*((char*)&_FlightData+i));
            blockBytes--;     //decrement the block space
      }
      Wire.endTransmission();
      delay(10);   //required write delay 5ms
      return eeaddress;
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
  //Serial.println(v_ret);
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
void logger_I2C_eeprom::setFlightRocketPos(long pos[4])
{
  /* _FlightData.w = pos[0];
    _FlightData.x = pos[1];
    _FlightData.y = pos[2];
    _FlightData.z = pos[3];*/
}
void logger_I2C_eeprom::getFlightRocketPos(long *pos) {
  /* pos[0] = _FlightData.w;
    pos[1] = _FlightData.x;
    pos[2] = _FlightData.y;
    pos[3] = _FlightData.z;*/
}
void logger_I2C_eeprom::setFlightCorrection( long OutputX, long OutputY)
{
  /*_FlightData.OutputX = OutputX;
    _FlightData.OutputY = OutputY;*/
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
/*long logger_I2C_eeprom::getFlightType(int flightNbr)
  {
  return  _FlightConfig[flightNbr].flight_type;
  }*/
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

      Serial1.println(String(currentTime) + "," + getFlightAltitudeData());

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
  //flight_type = getFlightType(flightNbr);
  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();
      long pos[4];
      getFlightRocketPos(pos);
      Serial1.println("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ";");

    }

  }
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

// returns 64, 32, 16, 8, 4, 2, 1, 0
// 0 is smaller than 1K
int logger_I2C_eeprom::determineSize()
{
  int rv = 0;  // unknown
  uint8_t orgValues[8];
  uint16_t addr;

  // try to read a byte to see if connected
  rv += _ReadBlock(0x00, orgValues, 1);

  if (rv == 0) return -1;

  // remember old values, non destructive
  for (uint8_t i = 0; i < 8; i++)
  {
    addr = (512 << i) + 1;
    orgValues[i] = read_byte(addr);
  }

  // scan page folding
  for (uint8_t i = 0; i < 8; i++)
  {
    rv = i;
    uint16_t addr1 = (512 << i) + 1;
    uint16_t addr2 = (512 << (i + 1)) + 1;
    write_byte(addr1, 0xAA);
    write_byte(addr2, 0x55);
    if (read_byte(addr1) == 0x55) // folded!
    {
      break;
    }
  }

  // restore original values
  for (uint8_t i = 0; i < 8; i++)
  {
    uint16_t addr = (512 << i) + 1;
    write_byte(addr, orgValues[i]);
  }
  return 0x01 << (rv - 1);
}
