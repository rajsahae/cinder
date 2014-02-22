#include "Cinder.h"

Cinder cinder;

void setup()
{
  cinder.begin();
  Serial.begin(115200);
}

void loop()
{
  cinder.setMotor1Speed(100);
  cinder.setMotor2Speed(100);
  delay(1000);
  cinder.setMotor2Speed(-100);
  cinder.setMotor1Speed(-100);
  delay(1000);
  cinder.setMotor1Speed(0);
  cinder.setMotor2Speed(0);
  delay(500);
}