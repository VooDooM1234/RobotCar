#include <Arduino.h>
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#line 23 "e:\\Projects\\RobotCar\\in\\Main.ino"
void setup();
#line 33 "e:\\Projects\\RobotCar\\in\\Main.ino"
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

// Robot directional control state machine
void DirectionControl::directionSelect(int direction)
{
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
    }
}

void DirectionControl::setCurrentRobotDirection(int d)
{
    robotDirection = d;
}

int DirectionControl::getCurrentRobotDirection()
{
    return robotDirection;
}

void DirectionControl::moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setCurrentRobotDirection(forward);
}

void DirectionControl::moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    setCurrentRobotDirection(reverse);
}

void DirectionControl::moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    setCurrentRobotDirection(right);
}
void DirectionControl::moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setCurrentRobotDirection(left);
}
void DirectionControl::stopMove()
{
    Serial.println("STOP");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    setCurrentRobotDirection(stop);
}

#line 1 "e:\\Projects\\RobotCar\\in\\Main.ino"
/**
 * @brief  Robot car with ultrasonic senor collision detection
 * @note   
 * @baudRate: 9600
 * @board: Arduino uno
 * @retval 
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
  directionControl.directionSelect(directionControl.direction::stop);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance();

  if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::left)
  {
    directionControl.directionSelect(directionControl.direction::left);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true && directionControl.getCurrentRobotDirection() != directionControl.direction::forward)
  {
    directionControl.directionSelect(directionControl.direction::forward);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::right)
  {
    directionControl.directionSelect(directionControl.direction::right);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::reverse)
  {
    directionControl.directionSelect(directionControl.direction::reverse);
  }
  

 
}

