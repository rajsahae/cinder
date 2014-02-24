#include "Cinder.h"

Cinder cinder;

void setup()
{
  cinder.begin();
  Serial.begin(115200);
}

void loop()
{
  Serial.println("motor1 forward");
  cinder.setMotor1Speed(100);
  delay(1000);
  Serial.println("motor2 forward");
  cinder.setMotor1Speed(0);
  cinder.setMotor2Speed(100);
  delay(1000);
  Serial.println("motor1 reverse");
  cinder.setMotor2Speed(0);
  cinder.setMotor1Speed(-100);
  delay(1000);
  Serial.println("motor2 reverse");
  cinder.setMotor1Speed(0);
  cinder.setMotor2Speed(-100);
  delay(1000);
  cinder.setMotor2Speed(0);
  
}
