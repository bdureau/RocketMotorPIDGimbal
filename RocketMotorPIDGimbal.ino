/*

   Description: Model Rocket motor gimbal using 2 servo. This is yet again another attempt to fly a model rocket without fins using long burn motors.
                It is using the Arduino PID controler to move a rocket motor.
                Angle changes can be monitored using a USB cable or a bluetooth interface
                Inspired by various camera gimbal projects
   Author: Boris du Reau
   Date: June 2018-2020
   Sensor used is an MPU6050 board

   You can use an Arduino Uno/Nano or stm32F103C board

   Servo Connection
   BROWN - gnd
   red - 5v
   yellow - d10 (pwm for Sero X) or PA1 for stm32
          - d11 (servo Y) or PA2 for stm32

   MPU board Connection
   VCC - 5v
   GND - GND
   SCL - A5  or pin SCL on the stm32
   SDA - A4  or pin SDA on the stm32
   INT - D2 (not used)

  This can be configured using the Gimbale Android application
  which is also hosted on Github
Version 1.0: 
Can log the data from all sensors on the eeprom and comunicate with an Android device 
via the serial port or a bluetooth module.  
Version 1.1
Configure the accelero and gyro range
Fixes, added checksum
*/


#include "config.h"
#include "global.h"
#include "utils.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"
logger_I2C_eeprom logger(0x50) ;
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start adress for the flights. Anything before that is the flight index
long currentMemaddress = 200;
boolean liftOff = false;
boolean landed = true;
//ground level altitude
long initialAltitude;
long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
/*
 * ReadAltitude()
 * Read altitude and filter any nose with a Kalman filter
 */
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

/*
   Initial setup
    do the board calibration
    if you use the calibration function do not move the board until the calibration is complete

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

  // set both servo's to 90 degree
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

  bmp.begin( config.altimeterResolution);

  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
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
  
  

  // INPUT CALIBRATED OFFSETS HERE; SPECIFIC FOR EACH UNIT AND EACH MOUNTING CONFIGURATION!!!!
  // use the calibrate function for yours
  // you can also write down your offset and use them so that you do not have to re-run the calibration
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
}

/*

   Initialize the MUP6050 sensor

*/
void initialize() {
  // verify connection
#ifdef SERIAL_DEBUG
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif
  // initialize device
  // do not use the constructor so that we can change the gyro and ACCEL RANGE
  // values accelerometer are:
  // MPU6050_ACCEL_FS_2
  // MPU6050_ACCEL_FS_4
  // MPU6050_ACCEL_FS_8
  // MPU6050_ACCEL_FS_16
  // Values for the Gyro are:
  // MPU6050_GYRO_FS_250
  // MPU6050_GYRO_FS_500
  // MPU6050_GYRO_FS_1000
  // MPU6050_GYRO_FS_2000
  
#ifdef SERIAL_DEBUG
  Serial1.println(F("Initializing MPU 6050 device..."));
#endif
  mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
 // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(config.gyroRange);
  mpu.setFullScaleAccelRange(config.acceleroRange);
  mpu.setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

  // load and configure the DMP
#ifdef SERIAL_DEBUG
  Serial1.println(F("Initializing DMP"));
#endif
  mpu.initialize(); //added 
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  //turn the PID on
  //servo's can go from 60 degree's to 120 degree so set the angle correction to + - 30 degrees max
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

void Mainloop(void)
{
  long startTime = millis();
  
  //read current altitude
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;
  if (config.liftOffDetect == 0) { //use baro detection
    if ( currAltitude > liftoffAltitude)
      lift = true;
  }
  else { // use accelero
    if (mpu.getAccelerationY() > 30000)
      lift = true;
  }

  if ((lift && !liftOff) || (recording && !liftOff))
  {
    liftOff = true;
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
    //Serial1.println("We have a liftoff");
  }
  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;
    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData(currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());

    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    logger.setFlightRocketPos((long) (w * 1000), (long) (q.x * 1000), (long) (q.y * 1000), (long) (q.z * 1000));
    logger.setFlightCorrection( (long) OutputX, (long)OutputY);
    logger.setAcceleration(mpu.getAccelerationX(), mpu.getAccelerationY(), mpu.getAccelerationZ());
    if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
      //flight is full let save it
      //save end address
      logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
      canRecord = false;
    } else {
      currentMemaddress = logger.writeFastFlight(currentMemaddress);
      currentMemaddress++;
    }
  }

  if (((canRecord && currAltitude < 10) && liftOff && !recording && !rec) || (!recording && rec))
  {
    liftOff = false;
    rec = false;
    //end loging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
   
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

  // display Euler angles in degrees
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
  SendTelemetry(q1, 200);
  checkBatVoltage(BAT_MIN_VOLTAGE);

  if (!liftOff) // && !canRecord)
    delay(10);

  // flush buffer to prevent overflow
 mpu.resetFIFO();
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[1000]="";


  while ( readVal != ';') {
    if(mainLoopEnable)
      Mainloop();
    while (Serial1.available())
    {
      readVal = Serial1.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }

  interpretCommandBuffer(commandbuffer);
for (int i=0; i< sizeof(commandbuffer); i++)
  commandbuffer[i]='\0';
}
/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  Different from other altimeters. Here it will calibrate the IMU
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   h  hello. Does not do much
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {
  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
#ifdef SERIAL_DEBUG
    Serial1.println(F("Erase\n"));
#endif
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
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
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    Serial1.println(F("Flight List: \n"));
    logger.printFlightList();
  }
  // calibrate the IMU
  else if (commandbuffer[0] == 'c')
  {
    //Serial1.println(F("calibration\n"));
    // Do calibration suff
    state = 0;
    calibrate();
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
    config.cksum = 0;//CheckSumConf(config);
    writeConfigStruc();
    Serial1.print(F("$OK;\n"));
  }
  //get all flight data
  else if (commandbuffer[0] == 'a')
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
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer))
      Serial1.print(F("$OK;\n"));
    else
      Serial1.print(F("$KO;\n"));
      commandbuffer="";
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    Serial1.print(F("$OK;\n"));
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
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
    Serial1.println(commandbuffer);
  }
}

/*

   Send telemetry to the Android device

*/
/*void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;
  //float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude() - initialAltitude;
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      Serial1.print(F("$telemetry,"));
      Serial1.print("RocketMotorGimbal");
      Serial1.print(F(","));
      //tab 1
      //GyroX
      Serial1.print(mpu.getRotationX());
      Serial1.print(F(","));
      //GyroY
      Serial1.print(mpu.getRotationZ());
      Serial1.print(F(","));
      //GyroZ
      Serial1.print(mpu.getRotationY());
      Serial1.print(F(","));
      //AccelX
      Serial1.print(mpu.getAccelerationX());
      Serial1.print(F(","));
      //AccelY
      Serial1.print(mpu.getAccelerationZ());
      Serial1.print(F(","));
      //AccelZ
      Serial1.print(mpu.getAccelerationY());
      Serial1.print(F(","));
      //OrientX
      Serial1.print(mpuYaw);
      Serial1.print(F(","));
      //OrientY
      Serial1.print(mpuRoll); //mpuPitch
      Serial1.print(F(","));
      //OrientZ
      Serial1.print(mpuPitch);//mpuRoll
      Serial1.print(F(","));

      //tab 2
      //Altitude
      Serial1.print(currAltitude);
      Serial1.print(F(","));
      //temperature
      Serial1.print(temperature);
      Serial1.print(F(","));
      //Pressure
      Serial1.print(pressure);
      Serial1.print(F(","));
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      int batVoltage = analogRead(PB1);
      // Serial1.print(batVoltage);
      float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
      Serial1.print(bat);
      Serial1.print(F(","));
      //tab3
      serialPrintFloatArr(arr, 4);
      //Serial1.print(F(","));
      Serial1.print((int)(100*((float)logger.getLastFlightEndAddress()/endAddress))); 
      Serial1.print(F(","));
      Serial1.print((int)correct);
      Serial1.print(F(","));
      Serial1.print(-mpuPitch + 180); // ServoX
      Serial1.print(F(","));
      //Serial1.print(mpuRoll + 90); //ServoY
      Serial1.print(mpuYaw + 90); //ServoY
      Serial1.println(F(";"));
    }
}*/

void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  long pressure;

  char myTelemetry[300]="";
  
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude() - initialAltitude;
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      strcat( myTelemetry , "telemetry,RocketMotorGimbal,");
      //tab 1
      //GyroX
      char temp[10];
      sprintf(temp, "%l", mpu.getRotationX());
      //dtostrf(mpu.getRotationX(), 4, 2, temp);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      //GyroY
      sprintf(temp, "%l", mpu.getRotationZ());
      strcat( myTelemetry , temp); 
      //dtostrf(mpu.getRotationZ(), 4, 2, temp);
      strcat( myTelemetry, ",");
      //GyroZ
      sprintf(temp, "%l", mpu.getRotationY());
      //dtostrf(mpu.getRotationY(), 4, 2, temp);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      //AccelX
      sprintf(temp, "%l", mpu.getAccelerationX());
      //dtostrf(mpu.getAccelerationX(), 4, 2, temp);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      //AccelY
      sprintf(temp, "%l", mpu.getAccelerationZ());
      strcat( myTelemetry , temp); 
      //dtostrf(mpu.getAccelerationZ(), 4, 2, temp);
      strcat( myTelemetry, ",");
      //AccelZ
      sprintf(temp, "%l", mpu.getAccelerationY());
      strcat( myTelemetry , temp); 
      //dtostrf(mpu.getAccelerationY(), 4, 2, temp);
      strcat( myTelemetry, ",");
      //OrientX
      sprintf(temp, "%l", (long)mpuYaw);
      strcat( myTelemetry , temp); 
      //dtostrf(mpuYaw, 4, 2, temp);
      strcat( myTelemetry, ",");
      //OrientY
      sprintf(temp, "%l", (long)mpuRoll);
      strcat( myTelemetry , temp); 
      //dtostrf(mpuRoll, 4, 2, temp);
      strcat( myTelemetry, ",");
      //OrientZ
      sprintf(temp, "%l", (long)mpuPitch);
      strcat( myTelemetry , temp); 
      //dtostrf(mpuPitch, 4, 2, temp);
      strcat( myTelemetry, ",");
      //tab 2
      //Altitude
      sprintf(temp, "%i",(int) currAltitude);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ","); 
      //temperature
      sprintf(temp, "%i",(int) temperature);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      //Pressure
      sprintf(temp, "%i", (int)pressure);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      int batVoltage = analogRead(PB1);
      float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
      sprintf(temp, "%f", bat);
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
      
      sprintf(temp, "%i", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)));
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
  
      sprintf(temp, "%i", (int)correct);
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");

      sprintf(temp, "%i",(int) ( -mpuPitch + 180));
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");

      sprintf(temp, "%i",(int) ( mpuYaw + 90));
      strcat( myTelemetry , temp); 
      strcat( myTelemetry, ",");
      
      unsigned int chk;
      chk = msgChk(myTelemetry, sizeof(myTelemetry));
      sprintf(temp, "%i", chk);
      strcat(myTelemetry,temp);
      strcat(myTelemetry, ";");
      Serial1.print("$");
      Serial1.println(myTelemetry);
    }
}

/*

   Send the Gimbal configuration to the Android device

*/
/*void SendAltiConfig() {
  bool ret = readAltiConfig();

  Serial1.print(F("$alticonfig"));
  Serial1.print(F(","));
  //AltimeterName
  Serial1.print("RocketMotorGimbal");
  Serial1.print(F(","));
  Serial1.print(config.ax_offset);
  Serial1.print(F(","));
  Serial1.print(config.ay_offset);
  Serial1.print(F(","));
  Serial1.print(config.az_offset);
  Serial1.print(F(","));
  Serial1.print(config.gx_offset);
  Serial1.print(F(","));
  Serial1.print(config.gy_offset);
  Serial1.print(F(","));
  Serial1.print(config.gz_offset);
  Serial1.print(F(","));
  Serial1.print(config.KpX);
  Serial1.print(F(","));
  Serial1.print(config.KiX);
  Serial1.print(F(","));
  Serial1.print(config.KdX);
  Serial1.print(F(","));
  Serial1.print(config.KpY);
  Serial1.print(F(","));
  Serial1.print(config.KiY);
  Serial1.print(F(","));
  Serial1.print(config.KdY);
  Serial1.print(F(","));
  Serial1.print(config.ServoXMin);
  Serial1.print(F(","));
  Serial1.print(config.ServoXMax);
  Serial1.print(F(","));
  Serial1.print(config.ServoYMin);
  Serial1.print(F(","));
  Serial1.print(config.ServoYMax);
  Serial1.print(F(","));
  Serial1.print(config.connectionSpeed);
  Serial1.print(F(","));
  Serial1.print(config.altimeterResolution);
  Serial1.print(F(","));
  Serial1.print(config.eepromSize);
  Serial1.print(F(","));
  //alti major version
  Serial1.print(MAJOR_VERSION);
  //alti minor version
  Serial1.print(F(","));
  Serial1.print(MINOR_VERSION);
  Serial1.print(F(","));
  Serial1.print(config.unit);
  Serial1.print(F(","));
  Serial1.print(config.endRecordAltitude);
  Serial1.print(F(","));
  Serial1.print(config.beepingFrequency);
  Serial1.print(F(","));
  Serial1.print(config.liftOffDetect);
  Serial1.print(F(","));
  Serial1.print(config.gyroRange);
  Serial1.print(F(","));
  Serial1.print(config.acceleroRange);
  Serial1.print(F(";\n"));
  
}
*/
void SendAltiConfig() {
  bool ret = readAltiConfig();
  char myconfig [300] = "";

  strcat(myconfig , "alticonfig,");
  //AltimeterName
  strcat(myconfig , "RocketMotorGimbal,");
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
  unsigned int chk = msgChk(myconfig, sizeof(myconfig));
  sprintf(temp, "%i", chk);
  strcat(myconfig, temp);
  strcat(myconfig, ";\n");
  Serial1.print("$");
  Serial1.print(myconfig);
}

/*


   calibration routines those will be executed each time the board is powered up.
   we might want to calibrate it for good on a flat table and save it to the microcontroler eeprom


*/
void calibrate() {
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
      calibration();
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
      //while (1);
      #endif
      break;
    }
  }
}

/*
   Used by the calibration fonction

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

void calibration() {
  int calibrationPass =0;
  /*ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;*/
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

    /*if (abs(mean_ay) <= acel_deadzone) ready++;
      else ay_offset = ay_offset - mean_ay / acel_deadzone;

      if (abs(16384 - mean_az) <= acel_deadzone) ready++;
      else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;*/
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

    if (ready == 6) break;
  }
}

/*

   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {

  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1);
 
  float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
  
  if (bat < minVolt) {
    for (int i = 0; i < 10; i++)
    {
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
}
