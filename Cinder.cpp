/*
   Cinder.cpp - Library for interacting with the Cinder robotics platform
   Created by Raj Sahae, Jan 26, 2014
   Released into the public domain

   The pins 8,9,10,11,12,13 are used by the Cinder library.
   */

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "TinyGPS.h"
#include "LiquidCrystal_I2C.h"

#include "Cinder.h"

Cinder::Cinder() :
  _HMC6352SlaveAddress(0x42 >> 1), // Compass address
  _HMC6352ReadAddress(0x41), //"A" in hex, compass read command
  _lcdAddress(0x27), // LCD address, 0x4F << 1
  _compassOffset(3), // physical offset of compass on board (degrees)
  _smcRxPin(10), // motor pins on Arduino
  _smcTxPin(11),
  _smcResetPin(12),
  _vmotControlPin(13),

  // Motor control bytes
  _motor1(Cinder::MOTOR1),
  _motor2(Cinder::MOTOR2),
  _startByte(0x80),
  _deviceType(0x00),
  _changeConfigByte(0x02),
  _configByte(B10), // binary 00000010, controlling motors 2 and 3

  // GPS: EM-406A
  _gpsRxPin(9),
  _gpsTxPin(8),

  _lcd(_lcdAddress, 20, 4), // 20x4 _lcd
  _serialGpsController(_gpsRxPin, _gpsTxPin),
  _serialMotorController(_smcRxPin, _smcTxPin)
{
  Serial.begin(115200);
}

void Cinder::begin()
{

  Wire.begin();
  _lcd.init();
  _lcd.clear();
  _lcd.backlight();
  _lcd.setCursor(0,0); // char 2, line 0
  _lcd.print("Cinder Lib v1.0");

  /*
     Somehow, configuring the motor controller is causing the GPS to completely fail.
     I have no idea why it's happening or how to fix it but I suspect it's an interference
     problem with the serial interface.
     */
  // Configure the motor controller at 9600 baud, dual motors = true, motor1 number = 2 (motor2 = 3)
  _lcd.setCursor(0,1);
  _lcd.print("Create motor control");
  configureMotorController(1200, true, 2);

  // configure gps serial
  _lcd.setCursor(0,2);
  _lcd.print("Create gps control");
  _serialGpsController.begin(4800);

  _lcd.setCursor(0,3);
  _lcd.print("Config motor control");

  // Pin to control power to the motors
  // Kill power to motors to ensure they don't move during setup
  pinMode(_vmotControlPin, OUTPUT);
  digitalWrite(_vmotControlPin, LOW);

  // You need to write the reset pin to low before you write it to high.
  pinMode(_smcResetPin, OUTPUT);
  digitalWrite(_smcResetPin, LOW);
  delay(200);
  digitalWrite(_smcResetPin, HIGH);
  delay(200);


  // we have to reset the controller after configuring it for it to work
  // delay 3 seconds to visually inspect the motor controller is correctly configured
  delay(200);
  digitalWrite(_smcResetPin, LOW);
  delay(200);
  digitalWrite(_smcResetPin, HIGH);
  delay(200);

  // Turn the motor power back on
  digitalWrite(_vmotControlPin, HIGH);
}

boolean Cinder::hasFix()
{
  acquireGps();

  if ( _currentAge == TinyGPS::GPS_INVALID_AGE )
    return false;
  else if ( _currentAge > 5000 )
    return false;
  else if ( _gps.hdop() > 5 )
    return false;
  else
    return true;
}

void Cinder::acquireGps()
{
  while(true)
  {

    // Pull in GPS data - loop back if we don't get any
    if(!feedgps()) continue;

    // Get our coordinates from GPS
    _gps.f_get_position(&_currentLat, &_currentLon, &_currentAge);

    // Error check the gps data
    if (_currentAge == TinyGPS::GPS_INVALID_AGE) continue;
    if (_currentLat == TinyGPS::GPS_INVALID_F_ANGLE) continue;
    if (_currentLon == TinyGPS::GPS_INVALID_F_ANGLE) continue;
    if (_gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES) continue;
    if (_gps.hdop() == TinyGPS::GPS_INVALID_HDOP) continue;

    break;
  }
}

void Cinder::driveTo(float toLat, float toLon)
{
  const int rate = 50; // delay rate in milliseconds

  _lcd.clear();
  _lcd.print("Pos:");
  _lcd.setCursor(0,1);
  _lcd.print("Heading:");
  _lcd.setCursor(0,2);
  _lcd.print("Bearing:");

  _lcd.setCursor(0,3);
  _lcd.print("Distance:");

  _lcd.setCursor(14,1);
  _lcd.print("M1:");
  _lcd.setCursor(14,2);
  _lcd.print("M2:");

  // Track location values between loops
  int previousLat = 0;
  int previousLon = 0;
  int previousHeading = -1;
  int previousDesiredHeading = -1;
  float previousDistance = -1.0;

  while (true)
  {
    acquireGps();

    if ( _currentLat != previousLat )
    {
      lcdClearAndPrint(5, 0, _currentLat);
      previousLat = _currentLat;
    }

    if (_currentLon != previousLon )
    {
      lcdClearAndPrint(11, 0, _currentLon);
      previousLon = _currentLon;
    }

    int currentHeading = readFromCompass();
    int desiredHeading = TinyGPS::course_to(_currentLat, _currentLon, toLat, toLon);
    float currentDistance = TinyGPS::distance_between(toLat, toLon, _currentLat, _currentLon);

    Serial.print("[driveTo] Location: (");
    Serial.print(_currentLat);
    Serial.print(", ");
    Serial.print(_currentLon);
    Serial.print(")");
    Serial.print(" | currentHeading: ");
    Serial.print(currentHeading);
    Serial.print(" | desireddHeading: ");
    Serial.print(desiredHeading);
    Serial.print(" | currentDistance: ");
    Serial.println(currentDistance);

    if ( currentDistance < 1 )
    {
      allStop();
      return;
    }

    if (currentHeading != previousHeading)
    {
      lcdClearAndPrint(8, 1, currentHeading);
      previousHeading = currentHeading;
    }

    if (desiredHeading != previousDesiredHeading)
    {
      lcdClearAndPrint(8, 2, desiredHeading);
      previousHeading = currentHeading;
    }

    if (currentDistance != previousDistance)
    {
      lcdClearAndPrint(9, 3, currentDistance);
      previousDistance = currentDistance;
    }

    int headingDelta = desiredHeading - currentHeading;
    // Possible cases here (headingDelta)
    // 1 -> 180 or -180 -> -359 == right turn
    // -1 -> -179 or 181 -> 359 == left turn
    // 0 = go straight
    if ( withinTwoDegrees(headingDelta) )
    {
      goStraight(currentDistance);
    }
    else if ( 0 < headingDelta && headingDelta <= 180 ) {
      turn(headingDelta);
    }
    else if ( -360 < headingDelta && headingDelta <= -180 ) {
      turn(headingDelta + 360);
    }
    else if ( 180 < headingDelta && headingDelta < 360 ) {
      turn(headingDelta - 360);
    }
    else if ( -180 < headingDelta && headingDelta < 0 ) {
      turn(headingDelta);
    }
    delay(rate);
  }
}

int Cinder::readFromCompass()
{
  //"Get Data. Compensate and Calculate New Heading"
  Wire.beginTransmission(_HMC6352SlaveAddress);
  Wire.write(_HMC6352ReadAddress);              // The "Get Data" command
  Wire.endTransmission();

  //time delays required by HMC6352 upon receipt of the command
  //Get Data. Compensate and Calculate New Heading : 6ms
  delay(6);

  Wire.requestFrom(_HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB

  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  // (MSB + LSB sum) / 10 - this should be a number between 0 and 360
  int heading = ((MSB << 8) + LSB) / 10;

  // Need to account for the physical offset of the compass
  heading -= _compassOffset;
  if (heading < 0) heading += 360;

  return heading;
}

boolean Cinder::feedgps()
{
  boolean encoded = false;
  while (_serialGpsController.available())
  {
    encoded = true;
    _gps.encode(_serialGpsController.read());
  }

  return encoded;
}

void Cinder::configureMotorController(int baud, boolean dual, byte motorNum)
{
  // Initialize motor controller serial communication
  _serialMotorController.begin(baud);

  // Delay for 100ms
  delay(100);

  // Configuration is achieved by sending a three-byte packet consisting of the start byte, a 
  // configuration command byte, and the new configuration byte:
  // start byte = 0x80, change configuration = 0x02, new settings, 0x00-0x7F
  _serialMotorController.write(_startByte);
  _serialMotorController.write(_changeConfigByte);

  byte configByte = 0;

  if (!dual) configByte = B01000000;

  configByte |= motorNum;

  _serialMotorController.write(configByte);
}

void Cinder::setMotorSpeed(int motor, int spd)
{
  Serial.print("[setMotorSpeed] motor: ");
  Serial.print(motor);
  Serial.print(" | speed: ");
  Serial.println(spd);

  // motorspeed can range from -127 to 127

  if (motor == _motor1) 
    lcdClearAndPrint(17, 1, spd/10);
  else if (motor == _motor2)
    lcdClearAndPrint(17, 2, spd/10);

  // To set the speed and direction of a motor, send a four-byte command with the following 
  // structure to the motor controller:
  // start byte = 0x80, device type = 0x00, motor # and direction, motor speed
  _serialMotorController.write(_startByte);

  // This byte identifies the device type for which the command is 
  // intended, and it should be 0x00 for commands sent to this motor controller. All devices 
  // that are not dual motor controllers ignore all subsequent bytes until another start byte is 
  // sent.
  _serialMotorController.write(_deviceType);

  byte motorByte = 0;

  if (spd < 0)
  {
    motorByte = 0; // OR 00000000
    spd = -spd;
  }
  else
  {
    motorByte = 1; // OR 00000001
  }

  if (spd > 127)
  {
    spd = 127;
  }

  motorByte |= ( motor << 1);

  _serialMotorController.write(motorByte);

  // The most significant bit must be zero since this is not a start 
  // byte. The remaining seven bits specify the motor speed. The possible range of values 
  // for byte 4 is thus 0x00 to 0x7F (0 to 127 decimal). 0x00 turns the motor off, and 0x7F 
  // turns the motor fully on; intermediate values correspond to intermediate speeds. The 
  // motor will brake when the speed is set to 0 in forward or reverse.
  _serialMotorController.write(spd);
}

void Cinder::lcdClearAndPrint(int col, int row, String msg)
{
  _lcd.setCursor(col, row);
  _lcd.print("   ");
  _lcd.setCursor(col, row);
  _lcd.print(msg);
}

void Cinder::lcdClearAndPrint(int col, int row, float num)
{
  _lcd.setCursor(col, row);
  _lcd.print("       ");
  _lcd.setCursor(col, row);
  _lcd.print(num);
}

void Cinder::lcdClearAndPrint(int col, int row, int num)
{
  _lcd.setCursor(col, row);
  _lcd.print("   ");
  _lcd.setCursor(col, row);
  _lcd.print(num);
}

void Cinder::goStraight(int error)
{
  // This is a PID control function for going straight.
  // The error is the distance to waypoint.
  
  Serial.print("[goStraight] error: ");
  Serial.println(error);

  int speed = 0;

  if (error < 5) {
    speed = 35;
  }
  else {
    speed = 127;
  }

  setMotorSpeed(_motor1, -speed);
  setMotorSpeed(_motor2, -speed);
}

void Cinder::turn(int error)
{
  // This is a PID conttrol function for turning.
  // The error is the difference in compass heading.

  static unsigned long lastTurnTime = 0;
  static int previousError = 0;
  static double I = 0;

  const float Kp = 127.0/180.0;
  const float Kd = 1E-6;
  const float Ki = 1E-7;

  unsigned long currentTurnTime = millis();

  float P = error;
  float D = (error - previousError)/(currentTurnTime - lastTurnTime);
  I += error/(currentTurnTime - lastTurnTime);

  float output = Kp*P + Kd*D + Ki*I;

  Serial.print("[turn] P: ");
  Serial.print(P);
  Serial.print(" | D: ");
  Serial.print(D);
  Serial.print(" | I: ");
  Serial.print(I);
  Serial.print(" | Kp*P: ");
  Serial.print(Kp*P);
  Serial.print(" | Kd*D: ");
  Serial.print(Kd*D);
  Serial.print(" | Ki*I: ");
  Serial.print(Ki*I);
  Serial.print(" | Output: ");
  Serial.println(output);

  setMotorSpeed(_motor1, (int)output);
  setMotorSpeed(_motor2, (int)-output);

  lastTurnTime = currentTurnTime;
  previousError = error;

}

boolean Cinder::withinTwoDegrees(int delta)
{
  // 1 -> 180 or -180 -> -359 == right turn
  // -1 -> -179 or 181 -> 359 == left turn
  // if delta == 1,2,-358, -359 (within 2 degrees of right turn)
  // if delta == -1, -2, 358, 359 (within 2 degrees of left turn)
  delta = abs(delta);
  if ( delta == 1 || delta == 2 || delta == 358 || delta == 359 )
    return true;
  else
    return false;

}

void Cinder::allStop()
{
  setMotorSpeed(_motor1, 0);
  setMotorSpeed(_motor2, 0);
}


void Cinder::setMotor1Speed(int spd) {
  setMotorSpeed(_motor1, spd);
}

void Cinder::setMotor2Speed(int spd) {
  setMotorSpeed(_motor2, spd);
}
