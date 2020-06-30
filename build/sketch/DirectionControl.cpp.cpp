#include <Arduino.h>
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#line 27 "e:\\Projects\\RobotCar\\in\\Main.ino"
void setup();
#line 39 "e:\\Projects\\RobotCar\\in\\Main.ino"
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
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    Serial.println("Motor setup complete");
}

// Robot directional control state machine
void DirectionControl::directionSelect(int direction)
{
    currentTimeRobot = millis();
    if (currentTimeRobot - previousTimeRobot >= robotMovementInterval)
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
        previousTimeRobot = currentTimeRobot;
    }
}
void DirectionControl::speedControl()
{
    analogWrite(ENA, robotSpeed);
    analogWrite(ENB, robotSpeed);
}

void DirectionControl::setRobotSpeed(int s)
{
    robotSpeed = s;
    speedControl();
}

int DirectionControl::getRobotSpeed()
{
    return robotSpeed;
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

#define MAX_SPEED 255
#define HALF_SPEED (MAX_SPEED/2) 

DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins;

bool flag = false;
unsigned long previousTimeFlag = 0;
int flagInterval = 1000;
unsigned long currentTimeFlag = 0;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  directionControl.direcetionSetup();
  directionControl.speedControl();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();
  directionControl.setRobotSpeed(255);
  directionControl.directionSelect(directionControl.direction::stop);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance();

  currentTimeFlag = millis();

  if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false && flag == true)
  {
    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::left);
    flag = false;
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false && flag == true)

  {

    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::right);
    flag = false;
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true && flag == true)
  {

    // directionControl.setRobotSpeed(HALF_SPEED);
    directionControl.directionSelect(directionControl.direction::forward);
    flag = false;
  }
  else if (/*ultraSonic.getDistance() <= 5*/ sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false)
  {
    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::reverse);
    flag = false;
  }

  if (currentTimeFlag - previousTimeFlag >= flagInterval)
  {
    flag = true;
  }
}

