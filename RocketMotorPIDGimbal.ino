/*
 * File:        RocketMotorPIDGimbalV1.3.ino
 * Project:     RocketMotorPIDGimbal V1.3
 * Author:      Boris du Reau
 * Date:        June 2018 - 2026
 * Description: Model rocket motor gimbal using 2 servos. This is yet
 *              another attempt to fly a model rocket without fins using
 *              long burn motors. The motor is moved according to the
 *              attitude computed by the MPU6050 DMP. The flight data is
 *              logged in an external I2C EEPROM and the gimbal can be
 *              configured and monitored with the Gimbal Android
 *              application (USB cable or bluetooth module), which is also
 *              hosted on Github. Inspired by various camera gimbal projects.
 *
 * Target:      STM32F103C board (Arduino Uno/Nano pin names in brackets)
 *
 * Wiring:
 *   Servos          BROWN  - GND
 *                   RED    - 5V
 *                   YELLOW - PA1 servo X (D10), PA2 servo Y (D11)
 *   MPU6050         VCC - 5V, GND - GND
 *                   SCL - SCL (A5), SDA - SDA (A4)
 *                   INT - PB12 (D2), required by the DMP
 *   Battery         PB1 through a voltage divider (see config.h)
 *   Buzzer          PA0
 *
 * Serial protocol: Serial1, 38400 bauds. Commands end with ';'. The
 *              message formats are shared with the Android application
 *              and must not be changed.
 *
 * History:
 *   Version 1.0 Logs the data from all sensors in the EEPROM and
 *               communicates with an Android device via the serial port
 *               or a bluetooth module.
 *   Version 1.1 Configurable accelerometer and gyroscope ranges.
 *               Fixes, added checksum.
 *   Version 1.2 Fixes.
 *   Version 1.3 Additional parameter for the telemetry module.
 *               Added the BMP280 sensor.
 *               Fixes and optimizations:
 *               - BMP085 compensation without floating point pow().
 *               - Telemetry no longer re-reads the altitude: the Kalman
 *                 filter is advanced only once per loop.
 *               - Bounded serial command buffer (no stack overflow).
 *               - The IMU calibration is no longer lost after a reboot
 *                 (configuration checksum fixed).
 *               - Accelerometer liftoff: the flight is no longer ended
 *                 immediately while the rocket is still on the pad.
 *               - A serial command received during a flight no longer
 *                 resets the flight in progress.
 *               - Apogee detection state is reset for every flight.
 *               - The configured accelerometer range is applied after
 *                 the DMP initialization, and the liftoff threshold
 *                 follows the range.
 *               - Configuration saved with a single flash erase.
 *               - The IMU calibration can no longer loop forever.
 *               - Correct battery voltage divider ratio.
 */


#include "config.h"
#include "global.h"
#include "utils.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"
logger_I2C_eeprom logger(0x50) ;
// size of the external EEPROM in bytes
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start address for the flights. Anything before that is the flight index
long currentMemaddress = 200;
boolean liftOff = false;
boolean rocketLanded = false;
boolean rocketApogee = false;
// true once the rocket has climbed above liftoffAltitude during the
// current flight. The landing detection is only armed after that, so that
// a flight started on the pad (accelerometer liftoff or manual recording)
// is not ended immediately.
boolean landingArmed = false;
//ground level altitude
long initialAltitude;
// altitude above which the barometric liftoff is detected (m)
long liftoffAltitude = 20;
// altitude below which the rocket is considered landed (m)
#define LANDING_ALTITUDE 10
long lastAltitude;
long apogeeAltitude;
//current altitude (above ground level, filtered)
long currAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
//nbr of measures to do so that we are sure that apogee has been reached
#define APOGEE_MEASURES 5
unsigned long measures = APOGEE_MEASURES;
long recordingTimeOut = 20000;
long lastBattWarning = 0;
// accelerometer liftoff threshold in raw counts, computed from
// LIFTOFF_ACCEL_THRESHOLD_G and the accelerometer range in initialize()
int32_t liftoffAccelThreshold = 30000;

#ifdef BMP085_180
/*
   ReadAltitude()
   Read the altitude from a BMP085 or BMP180 Bosch sensor and filter the
   noise with the Kalman filter.
   Warning: every call advances the Kalman filter, so call it only once
   per loop.
*/
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
#endif

#ifdef BMP280
/*
   ReadAltitude()
   Read the altitude from a BMP280 Bosch sensor and filter the noise with
   the Kalman filter.
   Warning: every call advances the Kalman filter, so call it only once
   per loop.
*/
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude(1013.25));
}
#endif

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/*
   setup()
   Initial setup: buzzer, servos, configuration, pressure sensor, Kalman
   filter, ground altitude, MPU6050 and flight list.
   The hard coded offsets below are only used when no valid configuration
   is found in the EEPROM.
*/
void setup()
{
  pinMode(pinSpeaker, OUTPUT);
  digitalWrite(pinSpeaker, LOW);
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);
  ax_offset = 1118;
  ay_offset = 513;
  az_offset = 1289;
  gx_offset = 64;
  gy_offset = -1;
  gz_offset = -33;
  ServoX.attach(PA1);  // attaches the X servo on PA1 for stm32 or D10 for the Arduino Uno
  ServoY.attach(PA2);  // attaches the Y servo on PA2 for stm32 or D11 for the Arduino Uno

  // set both servos to 90 degrees
  ServoX.write(90);
  ServoY.write(90);
  delay(500);

  Wire.begin();
  //default values
  //defaultConfig();
  // Read altimeter softcoded configuration
  boolean softConfigValid = false;
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    /*#ifdef SERIAL_DEBUG
        Serial1.println(F("Config invalid"));
        Serial1.println("ax_offset =" + config.ax_offset);
        Serial1.println("ay_offset =" + config.ay_offset);
        Serial1.println("az_offset =" + config.az_offset);
        Serial1.println("gx_offset =" + config.gx_offset);
        Serial1.println("gy_offset =" + config.gy_offset);
        Serial1.println("gz_offset =" + config.gz_offset);
      #endif*/
    //delay(1000);
    //calibrate();
    config.ax_offset = ax_offset;
    config.ay_offset = ay_offset;
    config.az_offset = az_offset;
    config.gx_offset = gx_offset;
    config.gy_offset = gy_offset;
    config.gz_offset = gz_offset;
    writeConfigStruc();
  }
  Serial1.begin(38400);
  while (!Serial1);      // wait for Leonardo enumeration, others continue immediately

  analogReadResolution(12); // 12 bit ADC, used by the battery voltage reading
  //bmp.begin( config.altimeterResolution);
  if (!bmp.begin( config.altimeterResolution)) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {
      shortBeep();
      delay(500);
      shortBeep();
      delay(500);
      longBeep();
      delay(500);
      }
  }

  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude readings
  // to initialize the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude(); //bmp.readAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);



  // Offsets are specific to each unit and each mounting configuration.
  // They come from the configuration saved by the calibration ('c' command).
  ax_offset = config.ax_offset;
  ay_offset = config.ay_offset;
  az_offset = config.az_offset;
  gx_offset = config.gx_offset;
  gy_offset = config.gy_offset;
  gz_offset = config.gz_offset;

  Serial1.println("init device");
  //Initialize MPU
  initialize();

  // Get flight
  int v_ret;
  v_ret = logger.readFlightList();
  //int epromsize = logger.determineSize();
  //Serial1.println(epromsize);
  long lastFlightNbr = logger.getLastFlightNbr();

  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
  //canRecord = true;
  recordingTimeOut = config.recordingTimeout * 1000;
}

/*
   accelRangeToLsbPerG()
   Return the MPU6050 accelerometer sensitivity (raw counts per g) for a
   range code: 0 = +/-2g, 1 = +/-4g, 2 = +/-8g, 3 = +/-16g.
*/
int32_t accelRangeToLsbPerG(int range) {
  if (range < 0 || range > 3)
    range = 0;
  return 16384L >> range;
}

/*
   applyAccelRange()
   Apply the configured accelerometer range and compute the matching
   accelerometer liftoff threshold in raw counts.
*/
void applyAccelRange() {
  int range = config.acceleroRange;
  if (range < 0 || range > 3)
    range = MPU6050_ACCEL_FS_2;
  mpu.setFullScaleAccelRange(range);
  liftoffAccelThreshold = (int32_t)(LIFTOFF_ACCEL_THRESHOLD_G * accelRangeToLsbPerG(range));
}

/*
   initialize()
   Initialize the MPU6050 sensor, its DMP and the PID controllers.

   Accelerometer range values (config.acceleroRange):
     MPU6050_ACCEL_FS_2, MPU6050_ACCEL_FS_4, MPU6050_ACCEL_FS_8,
     MPU6050_ACCEL_FS_16
   Gyroscope range values (config.gyroRange):
     MPU6050_GYRO_FS_250, MPU6050_GYRO_FS_500, MPU6050_GYRO_FS_1000,
     MPU6050_GYRO_FS_2000

   Note: mpu.initialize() and mpu.dmpInitialize() both reset the ranges,
   so the configured accelerometer range must be applied after them.
   The gyroscope range is NOT applied: the MotionApps 2.0 DMP firmware
   requires +/-2000 deg/s (set by dmpInitialize()) and its quaternion is
   wrong with any other range. config.gyroRange is kept in the
   configuration for the Android application only.
*/
void initialize() {
  // verify connection
#ifdef SERIAL_DEBUG
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

#ifdef SERIAL_DEBUG
  Serial1.println(F("Initializing MPU 6050 device..."));
#endif
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

  // load and configure the DMP
#ifdef SERIAL_DEBUG
  Serial1.println(F("Initializing DMP"));
#endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // apply the configured accelerometer range now that the DMP
  // initialization has reset it (gyro stays at +/-2000 deg/s, see above)
  applyAccelRange();
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  //turn the PID on
  //limit the PID correction to +/- 5 degrees
  myPIDX.SetOutputLimits(-5, 5);
  myPIDY.SetOutputLimits(-5, 5);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  SetpointX = 0;
  SetpointY = 0;
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
#ifdef SERIAL_DEBUG
    Serial1.println(F("Enabling DMP"));
#endif
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
#ifdef SERIAL_DEBUG
    Serial1.print(F("DMP Initialization failed code = "));
    Serial1.println(devStatus);
#endif
  }
}



/*

   MAIN PROGRAM LOOP

*/
void loop(void)
{
  MainMenu();
}

/*
   Mainloop()
   One iteration of the flight computer: altitude, liftoff, data logging,
   apogee and landing detection, attitude from the DMP, servo command,
   telemetry and battery check.
*/
void Mainloop(void)
{
  //read current altitude (the only Kalman filter update of the loop)
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;
  if (config.liftOffDetect == 0) { //use baro detection
    if ( currAltitude > liftoffAltitude)
      lift = true;
  }
  else { // use accelerometer, threshold depends on the accelerometer range
    if (mpu.getAccelerationY() > liftoffAccelThreshold)
      lift = true;
  }

  if ((lift && !liftOff) || (recording && !liftOff))
  {
    // start of a new flight
    liftOff = true;
    rocketLanded = false;
    rocketApogee = false;
    landingArmed = false;
    // reset the apogee detection
    lastAltitude = 0;
    measures = APOGEE_MEASURES;
    if (recording)
      rec = true;
    // save the time
    initialTime = millis();
    prevTime = 0;
    if (canRecord)
    {
      long lastFlightNbr = logger.getLastFlightNbr();

      if (lastFlightNbr < 0)
      {
        currentFileNbr = 0;
        currentMemaddress = 201;
      }
      else
      {
        currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
        currentFileNbr = lastFlightNbr + 1;
      }
      //Save start address
      logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
    }
  }

  // arm the landing detection once the rocket has really climbed
  if (liftOff && !landingArmed && currAltitude > liftoffAltitude)
    landingArmed = true;

  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;
    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData((long) currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());

    // quaternion stored as integers (x 1000)
    logger.setFlightRocketPos((long) (q.w * 1000), (long) (q.x * 1000), (long) (q.y * 1000), (long) (q.z * 1000));
    logger.setFlightCorrection( (long) OutputX, (long)OutputY);
    logger.setAcceleration(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
    if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
      //flight is full let save it
      //save end address
      logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
      canRecord = false;
    } else {
      delay(10);
      currentMemaddress = logger.writeFastFlight(currentMemaddress);
      // one spare byte between two records (EEPROM format, do not change)
      currentMemaddress++;
    }
  }

  //detect apogee: altitude decreasing for APOGEE_MEASURES consecutive loops
  if (liftOff && !rocketApogee) {
    if (currAltitude < lastAltitude  )
    {
      measures = measures - 1;
      if (measures == 0)
      {
        apogeeAltitude = lastAltitude;
        rocketApogee = true;
      }
    }
    else
    {
      lastAltitude = currAltitude;
      //number of measures to do to detect Apogee
      measures = APOGEE_MEASURES;
    }
  }

  // end of the flight recording when:
  // - the rocket is back below LANDING_ALTITUDE after having climbed
  //   above liftoffAltitude (automatic flight), or
  // - the manual recording has been stopped, or
  // - the recording timeout is reached
  if (((canRecord && currAltitude < LANDING_ALTITUDE) && liftOff && landingArmed && !recording && !rec) || (!recording && rec) || (canRecord && liftOff && !recording && !rec && (millis() - initialTime) > recordingTimeOut))
  {
    #ifdef SERIAL_DEBUG
    Serial1.println(F("finishing"));
    Serial1.println(currentFileNbr);
    #endif
    liftOff = false;
    rec = false;
    //end logging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
    if ((millis() - initialTime) > recordingTimeOut)
      canRecord = false;
  }

  if ((currAltitude < LANDING_ALTITUDE) && rocketApogee && !recording && !rec) {
    //landed
    rocketLanded = true;
  }

  /////////

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
#ifdef SERIAL_DEBUG
    Serial1.println(F("FIFO overflow!"));
#endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // compute the Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuPitch = ypr[PITCH] * 180 / M_PI;
    mpuRoll = ypr[ROLL] * 180 / M_PI;
    mpuYaw  = ypr[YAW] * 180 / M_PI;

    /*mpuRoll = -ypr[ROLL] * 180 / M_PI;
      mpuYaw  = -ypr[YAW] * 180 / M_PI;*/

    // flush buffer to prevent overflow
    mpu.resetFIFO();
  }
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  InputX = mpuPitch;
  myPIDX.Compute();
  InputY = mpuRoll;
  myPIDY.Compute();

  //if using PID do those
  //ServoX.write(-OutputX + 90);
  //ServoY.write(OutputY + 90);

  // if you do not want to use the PID
  /* ServoX.write(-mpuPitch + 90);
    ServoY.write(mpuRoll + 90);*/
  int SX = -mpuPitch + 180;
  if (SX < 80)
    SX = 80;
  if (SX > 100)
    SX = 100;
  int SY = mpuYaw + 90;

  if (SY < 80)
    SY = 80;
  if (SY > 100)
    SY = 100;

  ServoX.write(SX);
  ServoY.write(SY);
  /*delay(10);*/


  float q1[4];
  //mpu.dmpGetQuaternion(&q, fifoBuffer);

  q1[0] = q.w;
  q1[1] = q.x;
  q1[2] = q.y;
  q1[3] = q.z;

  //serialPrintFloatArr(q1, 4);
  currentTime = millis() - initialTime;
  SendTelemetry(q1, 200);
  checkBatVoltage(BAT_MIN_VOLTAGE);

  if (!liftOff) // && !canRecord)
    delay(10);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
}




//================================================================
// Main menu: run the main loop while reading the commands sent by the
// Android application, then interpret the command once ';' is received
//================================================================
void MainMenu()
{
  // static buffer: not allocated on the stack at every call. It is kept
  // fully cleared between two commands (see the end of this function).
  static char commandbuffer[COMMAND_BUFFER_SIZE];
  char readVal = ' ';
  int i = 0;

  while ( readVal != ';') {
    if (mainLoopEnable)
      Mainloop();
    while (Serial1.available())
    {
      readVal = Serial1.read();
      if (readVal != ';' )
      {
        // ignore new lines, and drop the characters that do not fit in
        // the buffer instead of overflowing it
        if (readVal != '\n' && i < (COMMAND_BUFFER_SIZE - 1))
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i] = '\0';
        // do not reset the flight state while a flight is in progress,
        // otherwise the flight being recorded is lost
        if (!liftOff)
          resetFlight();
        break;
      }
    }
  }

  interpretCommandBuffer(commandbuffer);
  // clear only the part of the buffer that has been used by this command
  // (characters + terminator) so that the buffer is all zeros again
  memset(commandbuffer, 0, i + 1);
}
/*
   interpretCommandBuffer()
   Interpret the menu commands. They can be typed in a serial terminal or
   sent by the Android application. Every command ends with ';'.

   Commands are as follows:
   a  get all flight data
   b  get the gimbal configuration
   c  calibrate the IMU (different from the other altimeters) and save
      the offsets
   d  reset the configuration to the default values
   e  erase all saved flights
   h  hello. Does not do much
   l  list all flights
   m  followed by 1 or 0: main loop on/off
   n  return the number of recorded flights in the EEPROM
   o  send a test message
   p  write one configuration parameter (in RAM)
   q  save the configuration in the EEPROM
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   s  write the whole configuration (obsolete, does nothing)
   w  followed by 1 or 0: start or stop recording
   x  delete the last flight
   y  followed by 1 or 0: telemetry on/off
*/
void interpretCommandBuffer(char *commandbuffer) {
  //get all flight data
  if (commandbuffer[0] == 'a')
  {
    Serial1.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }
    Serial1.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial1.print(F("$start;\n"));

    SendAltiConfig();

    Serial1.print(F("$end;\n"));
  }
  // calibrate the IMU
  else if (commandbuffer[0] == 'c')
  {
    // Do the calibration. It is done with the accelerometer at +/-2 g
    // because the calibration targets 16384 counts for 1 g.
    state = 0;
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    if (!calibrate()) {
      // no convergence: warn with the buzzer, the best offsets found
      // are still saved
      longBeep();
    }
    // restore the configured accelerometer range
    applyAccelRange();
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    config.ax_offset = ax_offset;
    config.ay_offset = ay_offset;
    config.az_offset = az_offset;
    config.gx_offset = gx_offset;
    config.gy_offset = gy_offset;
    config.gz_offset = gz_offset;
    // a valid checksum is required, otherwise the configuration (and the
    // calibration) is rejected at the next power up
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
    Serial1.print(F("$OK;\n"));
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
#ifdef SERIAL_DEBUG
    Serial1.println(F("Erase\n"));
#endif
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
    resetFlight();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    Serial1.print(F("$OK;\n"));
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    Serial1.println(F("Flight List: \n"));
    logger.printFlightList();
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("main Loop enabled\n"));
#endif
      mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("main loop disabled\n"));
#endif
      mainLoopEnable = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    Serial1.print(F("$start;\n"));
    Serial1.print(F("$nbrOfFlight,"));
    logger.readFlightList();
    Serial1.print(logger.getLastFlightNbr() + 1);
    Serial1.print(";\n");
    Serial1.print(F("$end;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {
    Serial1.print(F("$start;\n"));
    sendTestTram();
    Serial1.print(F("$end;\n"));
  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeAltiConfigV2(commandbuffer)) {
      Serial1.print(F("$OK;\n"));
    }
    else
      Serial1.print(F("$KO;\n"));
  }
  else if (commandbuffer[0] == 'q')
  {
    writeConfigStruc();
    Serial1.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      Serial1.print(F("$start;\n"));
      logger.printFlightData(atoi(temp));
      Serial1.print(F("$end;\n"));
    }
    else
      Serial1.println(F("not a valid flight"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    /* if (writeAltiConfig(commandbuffer))
       Serial1.print(F("$OK;\n"));
      else
       Serial1.print(F("$KO;\n"));
      commandbuffer = "";*/
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Start Recording\n"));
#endif
      recording = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Stop Recording\n"));
#endif
      recording = false;
      //store start and end address
    //logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    //logger.writeFlightList();
    }
    Serial1.print(F("$OK;\n"));
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    logger.eraseLastFlight();
    logger.readFlightList();
    long lastFlightNbr = logger.getLastFlightNbr();
    if (lastFlightNbr < 0)
    {
      currentFileNbr = 0;
      currentMemaddress = 201;
    }
    else
    {
      currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
      currentFileNbr = lastFlightNbr + 1;
    }
    canRecord = logger.CanRecord();
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Telemetry enabled\n"));
#endif
      telemetryEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      Serial1.print(F("Telemetry disabled\n"));
#endif
      telemetryEnable = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
    Serial1.println(commandbuffer);
  }
}

/*
   SendTelemetry()
   Send the telemetry message to the Android device every "freq" ms.
   "arr" is the attitude quaternion (w, x, y, z).

   The altitude sent is the one already computed by Mainloop() (global
   currAltitude): calling ReadAltitude() here would advance the Kalman
   filter a second time in the same loop and cost one more sensor read.
*/
void SendTelemetry(float * arr, int freq) {

  float temperature;
  long pressure;

  char myTelemetry[220] = "";

  if (telemetryEnable && (millis() - last_telem_time) > freq) {
    last_telem_time = millis();
    pressure = bmp.readPressure();
    temperature = bmp.readTemperature();

    strcat( myTelemetry , "telemetry,");
    strcat( myTelemetry , BOARD_FIRMWARE);
    strcat( myTelemetry , ",");
    //tab 1
    //GyroX
    char temp[15];
    sprintf(temp, "%i", mpu.getRotationX());
    //dtostrf(mpu.getRotationX(), 4, 2, temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //GyroY
    sprintf(temp, "%i", mpu.getRotationZ());
    strcat( myTelemetry , temp);
    //dtostrf(mpu.getRotationZ(), 4, 2, temp);
    strcat( myTelemetry, ",");
    //GyroZ
    sprintf(temp, "%i", mpu.getRotationY());
    //dtostrf(mpu.getRotationY(), 4, 2, temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //AccelX
    sprintf(temp, "%i", mpu.getAccelerationX());
    //dtostrf(mpu.getAccelerationX(), 4, 2, temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //AccelY
    sprintf(temp, "%i", mpu.getAccelerationZ());
    strcat( myTelemetry , temp);
    //dtostrf(mpu.getAccelerationZ(), 4, 2, temp);
    strcat( myTelemetry, ",");
    //AccelZ
    sprintf(temp, "%i", mpu.getAccelerationY());
    strcat( myTelemetry , temp);
    //dtostrf(mpu.getAccelerationY(), 4, 2, temp);
    strcat( myTelemetry, ",");
    //OrientX
    sprintf(temp, "%i", (long)mpuYaw);
    strcat( myTelemetry , temp);
    //dtostrf(mpuYaw, 4, 2, temp);
    strcat( myTelemetry, ",");
    //OrientY
    sprintf(temp, "%i", (long)mpuRoll);
    strcat( myTelemetry , temp);
    //dtostrf(mpuRoll, 4, 2, temp);
    strcat( myTelemetry, ",");
    //OrientZ
    sprintf(temp, "%i", (long)mpuPitch);
    strcat( myTelemetry , temp);
    //dtostrf(mpuPitch, 4, 2, temp);
    strcat( myTelemetry, ",");
    //tab 2
    //Altitude
    sprintf(temp, "%i", (int) currAltitude);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //temperature
    sprintf(temp, "%i", (int) temperature);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //Pressure
    sprintf(temp, "%i", (int)pressure);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //Batt voltage
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    // 12 bit ADC, 3.3 V reference
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    dtostrf(bat, 4, 2, temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    //tab3
    floatToByte(arr[0], temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    floatToByte(arr[1], temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    floatToByte(arr[2], temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");
    floatToByte(arr[3], temp);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");

    //sprintf(temp, "%i", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)));
    if (!recording) {
      sprintf(temp, "%i", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) );
    }
    else {
      sprintf(temp, "%i", (int)(100 * ((float) currentMemaddress / endAddress)) );
    }
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");


    sprintf(temp, "%i", (int)correct);
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");

    sprintf(temp, "%i", (int) ( -mpuPitch + 180));
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");

    sprintf(temp, "%i", (int) ( mpuYaw + 90));
    strcat( myTelemetry , temp);
    strcat( myTelemetry, ",");

    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;
    sprintf(temp, "%i,", li);
    strcat(myTelemetry, temp);

    //check apogee
    int ap = 0;
    if (rocketApogee)
      ap = 1;
    sprintf(temp, "%i,", ap);
    strcat(myTelemetry, temp);

    sprintf(temp, "%i,", apogeeAltitude);
    strcat(myTelemetry, temp);


    int landed = 0;
    if (rocketLanded)
      landed = 1;
    sprintf(temp, "%i,", landed);
    strcat(myTelemetry, temp);

    sprintf(temp, "%i,", currentTime);
    strcat(myTelemetry, temp);

    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(myTelemetry, temp);

    unsigned int chk;
    chk = msgChk(myTelemetry, sizeof(myTelemetry));
    sprintf(temp, "%i", chk);
    strcat(myTelemetry, temp);
    strcat(myTelemetry, ";");
    Serial1.print("$");
    Serial1.println(myTelemetry);
  }
}

/*
   SendAltiConfig()
   Send the gimbal configuration to the Android device
*/

void SendAltiConfig() {
  bool ret = readAltiConfig();
  char myconfig [300] = "";

  strcat(myconfig , "alticonfig,");
  //AltimeterName
  strcat(myconfig , BOARD_FIRMWARE);
  strcat(myconfig , ",");
  char temp [10];
  sprintf(temp, "%i", config.ax_offset);
  strcat( myconfig , temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ay_offset);
  strcat( myconfig , temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.az_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gx_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gy_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gz_offset);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KpX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KiX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KdX);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KpY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KiY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.KdY);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoXMin);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoXMax);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoYMin);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.ServoYMax);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.connectionSpeed);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.altimeterResolution);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.eepromSize);
  strcat( myconfig, temp);
  strcat( myconfig, ",");
  //alti major version
  sprintf(temp, "%i", MAJOR_VERSION);
  strcat( myconfig, temp);
  strcat( myconfig, ",");
  //alti minor version
  sprintf(temp, "%i", MINOR_VERSION);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.unit);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.endRecordAltitude);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.beepingFrequency);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.liftOffDetect);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.gyroRange);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.acceleroRange);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.recordingTimeout);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.batteryType);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  sprintf(temp, "%i", config.telemetryType);
  strcat( myconfig, temp);
  strcat( myconfig, ",");

  unsigned int chk = msgChk(myconfig, sizeof(myconfig));
  sprintf(temp, "%i", chk);
  strcat(myconfig, temp);
  strcat(myconfig, ";\n");
  Serial1.print("$");
  Serial1.print(myconfig);
}

/*
   calibrate()
   IMU calibration routine, started by the 'c' command. The board must lie
   still with the rocket axis (Y) vertical. The offsets found are saved in
   the microcontroller EEPROM by the caller.
   Return false if the calibration did not converge within
   CALIBRATION_MAX_PASSES passes (the best offsets found are kept).
*/
bool calibrate() {
  bool converged = true;
#ifdef SERIAL_DEBUG
  // start message
  Serial1.println("\nMPU6050 Calibration Sketch");
  //delay(1000);
  Serial1.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  //delay(1000);
  // verify connection
  Serial1.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif
  //delay(1000);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
#ifdef SERIAL_DEBUG
      Serial1.println("\nReading sensors for first time...");
#endif
      meansensors();
      state++;
      delay(100);
    }

    if (state == 1) {
#ifdef SERIAL_DEBUG
      Serial1.println("\nCalculating offsets...");
#endif
      converged = calibration();
      state++;
      delay(100);
    }

    if (state == 2) {
      meansensors();
#ifdef SERIAL_DEBUG
      Serial1.println("\nFINISHED!");
      Serial1.print("\nSensor readings with offsets:\t");
      Serial1.print(mean_ax);
      Serial1.print("\t");
      Serial1.print(mean_ay);
      Serial1.print("\t");
      Serial1.print(mean_az);
      Serial1.print("\t");
      Serial1.print(mean_gx);
      Serial1.print("\t");
      Serial1.print(mean_gy);
      Serial1.print("\t");
      Serial1.println(mean_gz);
      Serial1.print("Your offsets:\t");
      Serial1.print(ax_offset);
      Serial1.print("\t");
      Serial1.print(ay_offset);
      Serial1.print("\t");
      Serial1.print(az_offset);
      Serial1.print("\t");
      Serial1.print(gx_offset);
      Serial1.print("\t");
      Serial1.print(gy_offset);
      Serial1.print("\t");
      Serial1.println(gz_offset);
      Serial1.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial1.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial1.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
#endif
      break;
    }
  }
  return converged;
}

/*
   meansensors()
   Used by the calibration function: average "buffersize" raw readings of
   the accelerometer and gyroscope (the first 100 readings are discarded).
*/
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

/*
   calibration()
   Used by the calibration function: iteratively adjust the offsets until
   the averaged readings are within the dead zones. The expected readings
   are 0 on X and Z and 16384 (1 g at +/-2 g) on Y, the rocket axis.
   Return false if it did not converge within CALIBRATION_MAX_PASSES.
*/
bool calibration() {
  int calibrationPass = 0;
  // first guess of the offsets
  ax_offset = -mean_ax / 8;
  ay_offset = (16384 - mean_ay ) / 8;
  az_offset = ( - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    calibrationPass ++;

    Serial1.print("$CAL_PASS ");
    Serial1.println(calibrationPass);
#ifdef SERIAL_DEBUG
    Serial1.println("...");
#endif
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(16384 - mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset + (16384 - mean_ay) / acel_deadzone;

    if (abs( mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + ( - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6)
      return true;

    // give up instead of looping forever (sensor moving or faulty)
    if (calibrationPass >= CALIBRATION_MAX_PASSES)
      return false;
  }
}

/*
   checkBatVoltage()
   Check every 10 s if the battery voltage is OK.
   If not, warn the user so that the battery does not get
   damaged by over discharging.
*/
void checkBatVoltage(float minVolt) {
  if ((millis() - lastBattWarning) > 10000) {
    lastBattWarning = millis();
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);

    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

    if (bat < minVolt) {
      //Serial1.println(bat);
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
  }
}

/*
   resetFlight()
   Re-initialize all the flight related global variables
*/
void resetFlight() {
  liftOff = false;
  apogeeAltitude = 0;
  rocketLanded = false;
  rocketApogee = false;
  landingArmed = false;
  // reset the apogee detection
  lastAltitude = 0;
  measures = APOGEE_MEASURES;
  logger.readFlightList();
  long lastFlightNbr = logger.getLastFlightNbr();
  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
}

/*
   sendTestTram()
   Send a test message to check the link with the Android application
*/
void sendTestTram() {

  char altiTest[100] = "";
  char temp[10] = "";

  strcat(altiTest, "testTrame," );
  strcat(altiTest, "Bear altimeters are the best!!!!,");
  unsigned int chk;
  chk = msgChk(altiTest, sizeof(altiTest));
  sprintf(temp, "%i", chk);
  strcat(altiTest, temp);
  strcat(altiTest, ";\n");
  Serial1.print("$");
  Serial1.print(altiTest);

}
