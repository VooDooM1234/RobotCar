#include <Arduino.h>
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#line 19 "e:\\Projects\\RobotCar\\in\\Main.ino"
void setup();
#line 29 "e:\\Projects\\RobotCar\\in\\Main.ino"
void loop();
#line 0 "e:\\Projects\\RobotCar\\in\\Main.ino"
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#include "DirectionControl.h"
#include "Pins.h"

void DirectionControl::direcetionSetup()
{
    Serial.println("Motor setup starting");
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Serial.println("Motor setup complete");
}
void DirectionControl::directionSelect(int direction){
    switch (direction)
    {
    case forward:
    moveForward();
        break;
     case reverse:
    moveBackwards();
        break;

         case right:
    moveRight();
        break;

         case left:
    moveLeft();
        break;

        case stop:
    stopMove();
        break;
        

    default:
    stopMove();
        break;
        
    }
}

void DirectionControl::moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void DirectionControl::moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void DirectionControl::moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
void DirectionControl::moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void DirectionControl::stopMove()
{
    Serial.println("STOP");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

#line 1 "e:\\Projects\\RobotCar\\in\\Main.ino"
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

