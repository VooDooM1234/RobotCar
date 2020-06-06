/*
    AVR Robot car code
*/
#include "DirectionControl.h"
#include "SensorServo.h"
#include "SensorUltraSonic.h"
#include "Pins.h"

DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins;

const int stopMovingInterval = 500;
int motorUpdate = 0;
bool goFlag = true;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  directionControl.direcetionSetup();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();
  directionControl.directionSelect(directionControl.direction::forward);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance(sensorServo.getIsServoMovementComplete());

  Serial.println("Is Clear?: ");
  Serial.println(ultraSonic.isClear(ultraSonic.getDistance()));

  Serial.print("Get Distance: ");
  Serial.println(ultraSonic.getDistance());

  if (ultraSonic.isClear(ultraSonic.getDistance()) == true && goFlag == true)
  {
    directionControl.directionSelect(directionControl.direction::forward);
  }
  else
  {
    directionControl.directionSelect(directionControl.direction::stop);
    goFlag = false;
  }

  if (millis() >= motorUpdate + stopMovingInterval)
  {
    motorUpdate += stopMovingInterval;
    goFlag = true;
  }
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}
