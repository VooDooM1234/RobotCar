/*
    AVR Robot car code
*/
#include "DirectionControl.h"
#include "SensorDistance.h"

#include "Pins.h"
#include <Servo.h>

DirectionControl directionControl = DirectionControl();
SensorDistance sensorDistance = SensorDistance();

Servo sonarServo;
Pins _pins;

int distance = 0;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  directionControl.direcetionSetup();
  sensorDistance.ultraSonicSetup();

  sonarServo.attach(_pins.sonarMotorPin);
  ServoMoveCentre();
}

void loop()
{

  sensorDistance.measureDistance();
  delay(500);
  servoMoveLeft();
  sensorDistance.measureDistance();
  delay(500);
  servoMoveRight();
  sensorDistance.measureDistance();
  delay(500);


  //directionControl.directionSelect(directionControl.direction::forward);
  // delay(1000);
  // directionControl.directionSelect(directionControl.direction::stop);
  // delay(1000);
}

void servoMoveLeft()
{
  sonarServo.write(0);
  delay(500);
}
void servoMoveRight()
{
  sonarServo.write(180);
  delay(500);
}
void ServoMoveCentre()
{
   sonarServo.write(90);
  delay(500);
}
