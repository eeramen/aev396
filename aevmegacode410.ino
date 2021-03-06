/***************************************************************************
 * Arduino Mega Code for AEV
 * by: Michelle Masutani, with assitance from Ricky Choi
 * date: April 9, 2018
 * 
 * This code includes: 1 PS2 remote control for the vehicle, 2 temperature
 * sensor, 3 voltmeters (for 48v rail, 12v buck converter, and 5 v ), 
 * 1 MPU 9250, and 1 LCD readout screen that prints sensor data. 
 * Sensor data is also printed to the Serial Monitor.
 * 
 * MUST ADD to this code: 1 wireless remote, 2 lights, and 
 *                        Raspberry Pi connections.
 *          MUST TEST GPS
 * 
 ***************************************************************************/ 

// include libraries

//MPU
#include "quaternionFilters.h"
#include "MPU9250.h"
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
// #define debug
//LCD
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
//PS2
#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#define PS2_DAT        13      
#define PS2_CMD        12  
#define PS2_SEL        11  
#define PS2_CLK        10  
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false
//GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define GPSECHO  true
//Running average
#include <runningaverage.h>

// class
PS2X ps2x; 
MPU9250 myIMU;
Adafruit_LiquidCrystal lcd(0);
Servo right1;         // attached to pin 9
Servo right2;         // attached to piun 8
Servo left1;          // attached to pin 7
Servo left2;          // attached to pin 6
RunningAverage batRA(10);
RunningAverage escRA(10);
RunningAverage arduinoRA(10);
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);

// create variables
int lcdTimeout = 4000; // lcd page timeout
int previousMillis = 0; // timer
int page  = 0; // lcd page
// ps2 setting
int error = 0; 
byte type = 0; 
byte vibrate = 0; 
// servo storage variables
int ly = 0;
int ry = 0;
int lyy = 0;
int ryy = 0;
int samples = 0;
float tempbat;// battery temp
float tempesc;// esc temp
float temparduino;
float volt48;
float volt12;
float volt5;
float tempbatAN;// battery temp
float tempescAN;// esc temp
float temparduinoAN;
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup()
{
  // setup lcd
  lcd.begin(20, 4);
  lcd.clear();
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  // setup ps2
    //servos
    right1.attach(9);
    left1.attach(8);
    right2.attach(7);
    left2.attach(6);
    //check for controller error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (error != 0){
      Serial.print("Error");
      lcd.print("PS2 Error"); delay(10000);}


#ifdef GPSchip
// setup GPS
GPS.begin(9600);
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
GPS.sendCommand(PGCMD_ANTENNA);
useInterrupt(true);
delay(1000);
// Ask for firmware version
mySerial.println(PMTK_Q_RELEASE);
#endif

// analog pins setup
pinMode(A0,INPUT);//tempbat
pinMode(A1,INPUT);//tempesc
pinMode(A2,INPUT);//temparduino
pinMode(A3,INPUT);//48v
pinMode(A4,INPUT);//12v
pinMode(A5,INPUT);//5v

// setup MPU
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
     
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    lcd.setCursor(0,1);
    lcd.print("MPU9250 is online");
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");


    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    lcd.setCursor(0,0);
    lcd.print("No connection: ");
    lcd.setCursor(0,1);
    lcd.print("MPU9250");
    // while (1) ; // Loop forever if communication doesn't happen
  }

//Serial3.println(F("Date, Time, Latitude, Longitude, Temp, Dissolved Oxygen, Electric Conductivity, pH "));

  
}//setup

#ifdef GPSchip
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t timer = millis();
#endif

void loop()
{  
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
      if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      // find y-axis values
      lyy = ps2x.Analog(PSS_LY);
      ryy = ps2x.Analog(PSS_RY);
      ly = map(lyy,0,255,1700,800);
      ry = map(ryy,0,255,1700,800);
      // write the values to the OUTPUTS 
      right1.writeMicroseconds(ry);
      left1.writeMicroseconds(ly);
      right2.writeMicroseconds(ry);
      left2.writeMicroseconds(ly);
      delay(15);
      Serial.print("Stick Values:");
      Serial.print(ly, DEC); 
      Serial.print(" left");
      Serial.print(",");
      Serial.print(ry, DEC); 
      Serial.println(" right");
      
      } // ps2

  
 
  //mpu
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  }
  // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);


    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per second independent of read rate
    if (myIMU.delt_t > 1000)
    {
      myIMU.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                  *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                          - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
      myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                  *(getQ() + 2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                                  *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                          - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 1000)

  // lcd timer
  int currentMillis = millis();

// temperature sensors
    tempbatAN = analogRead(A0);
    batRA.addValue(tempbatAN);
    tempescAN = analogRead(A1);
    escRA.addValue(tempescAN);
   //temparduino = (analogRead(A2)*0.48828125*9/5) +32;
 
 
 // voltmeters 
  volt48 = analogRead(A3)*55/1024;
  //Serial.print("48 rail: ");
  //Serial.println(volt48);
  volt12 = analogRead(A4)*20/1024;
  //Serial.print("12 rail: ");
  //Serial.println(volt12);
  volt5 = analogRead(A5)*1.84*5/1024;
  //Serial.print("5 rail: ");
  //Serial.println(volt5);
  
// average analog values
      samples++;
    if (samples == 100){
      samples = 0;
      tempbat = (batRA.getAverage()*0.48828125*9/5) +32;
      batRA.clear();
      tempesc = (escRA.getAverage()*0.48828125*9/5) +32;
      escRA.clear();
      }

#ifdef GPSchip
//GPS
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

#endif
// print to LCD

  if (currentMillis-previousMillis > lcdTimeout)
  {
    previousMillis = currentMillis;
    lcd.clear();
 
    switch (page) {
      case 0:
        lcd.setCursor(0,0);
        lcd.print("Acceleration in mg  ");
        
        lcd.setCursor(0, 1);
        lcd.print("X  ");
        lcd.print("    Y  ");
        lcd.print("    Z     ");
    
        lcd.setCursor(0, 2);
        lcd.print(1000 * myIMU.ax);

        
        lcd.setCursor(6,2);
        lcd.print(",");
        lcd.print(1000 * myIMU.ay);
        lcd.setCursor(13,2);
        lcd.print(",");
    
        lcd.print(1000 * myIMU.az);

        page = 1;
        break;
      case 1:
        lcd.setCursor(0,0);
        lcd.print("Gyro rate in deg/s");
    
        lcd.setCursor(0, 1);
        lcd.print("X  ");
        lcd.print("    Y  ");
        lcd.print("    Z     ");
    
        lcd.setCursor(0, 2);
        lcd.print(myIMU.gx);
        lcd.setCursor(6,2);
        lcd.print(",");
        lcd.print(myIMU.gy);
        lcd.setCursor(13,2);
        lcd.print(",");
      
        lcd.print(myIMU.gz);

        page = 2;
        break;
      case 2:
      lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Magnetic field in mG");
   
        lcd.setCursor(0, 1);
        lcd.print("X  ");
        lcd.print("    Y  ");
        lcd.print("    Z     ");
    
        lcd.setCursor(0,2);
        lcd.print(myIMU.mx);
        lcd.setCursor(6,2);
        lcd.print(",");
        lcd.print(myIMU.my);
        lcd.setCursor(13,2);
        lcd.print(",");
    
        lcd.print(myIMU.mz);

        page = 3;
        break;
      case 3:
        lcd.setCursor(0,0);
        lcd.print("Arduino: ");
        lcd.print(" *F");
        
        lcd.setCursor(0,1);
        lcd.print("Battery: ");
        lcd.print(tempbat,1);
        lcd.print(" *F");

        lcd.setCursor(0,2);
        lcd.print("ESC: ");
        lcd.print(tempesc,1);
        lcd.print(" *F");

        page = 4;
        break;
        
      case 4:
        lcd.setCursor(0,0);
        lcd.print("GPS location");

        page = 5;
        break;
      case 5:
        lcd.setCursor(0,0);
        lcd.print("48V rail: ");
        lcd.print(volt48);
        lcd.setCursor(0,1);
        lcd.print("12V rail: ");
        lcd.print(volt12);
        lcd.setCursor(0,2);
        lcd.print("5V rail: ");
        lcd.print(volt5);
    
        page = 5;
        break;
        
    }// switch (page)
  }//if(currentmillis-prevmillis

}
