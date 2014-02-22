/*
Cinder.h - Library for interacting with the Cinder robotics platform
Created by Raj Sahae, Jan 26, 2014
Released into the public domain

The pins 8,9,10,11,12,13 are used by the Cinder library.
*/

#ifndef Cinder_h
#define Cinder_h

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "TinyGPS.h"
#include "LiquidCrystal_I2C.h"

class Cinder
{
  public:
  
    const static int MOTOR1 = B10;
    const static int MOTOR2 = B11;
    
    Cinder();
    void begin();
    void acquireGps();
    void driveTo(float, float);
    boolean hasFix();
    void setMotor1Speed(int spd);
    void setMotor2Speed(int spd);
    
    float currentLat() { return _currentLat; }
    float currentLon() { return _currentLon; }
    unsigned long currentAge() { return _currentAge; }
    unsigned long hdop() { return _gps.hdop(); }
    
  private:
      
    float _currentLat;
    float _currentLon;
    unsigned long _currentAge;
    
    const int _HMC6352SlaveAddress; // Compass address
    const int _HMC6352ReadAddress; //"A" in hex, compass read command

    const int _lcdAddress; // LCD address, 0x4F << 1

    LiquidCrystal_I2C _lcd; // 20x4 lcd

    // physical offset of compass on board (degrees)
    const int _compassOffset;

    // motor pins on Arduino
    const int _smcRxPin;
    const int _smcTxPin;
    const int _smcResetPin;
    const int _vmotControlPin;

    // Motor control bytes
    const byte _motor1;
    const byte _motor2;
    const byte _startByte;
    const byte _deviceType;
    const byte _changeConfigByte;
    const byte _configByte; // binary 00000010, controlling motors 2 and 3

    // GPS: EM-406A
    TinyGPS _gps;
    const int _gpsRxPin;
    const int _gpsTxPin;

    SoftwareSerial _serialGpsController;
    SoftwareSerial _serialMotorController;

    void gpsdump(TinyGPS);
    boolean feedgps();
    int readFromCompass();

    void configureMotorController(int, boolean, byte);
    void setMotorSpeed(int, int);

    void goStraight(int speed);
    void turn(int error);
    void allStop();
    boolean withinTwoDegrees(int);

    void lcdClearAndPrint(int, int, String);
    void lcdClearAndPrint(int, int, int);
    void lcdClearAndPrint(int, int, float);
};

#endif