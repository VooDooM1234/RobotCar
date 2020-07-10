#include <Arduino.h>
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#line 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
#include "DirectionControl.h"
#include "Pins.h"

Pins pins;

#line 6 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void direcetionSetup();
#line 19 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void speedControl(int speed);
#line 28 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void directionSelect(int direction);
#line 53 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void setCurrentRobotDirection(int d);
#line 58 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
int getCurrentRobotDirection();
#line 63 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void moveForward();
#line 73 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void moveBackwards();
#line 83 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void moveRight();
#line 92 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void moveLeft();
#line 101 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void stopMove();
#line 27 "e:\\Projects\\RobotCar\\in\\Main.ino"
void setup();
#line 42 "e:\\Projects\\RobotCar\\in\\Main.ino"
void loop();
#line 6 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
void direcetionSetup()
{
    Serial.println("Motor setup starting");
    pinMode(pins.IN1, OUTPUT);
    pinMode(pins.IN2, OUTPUT);
    pinMode(pins.IN3, OUTPUT);
    pinMode(pins.IN4, OUTPUT);
    Serial.println("Motor setup complete");

    pinMode(pins.ENA, OUTPUT);
    pinMode(pins.ENB, OUTPUT);
}

void speedControl(int speed)
{   
    speed = map(speed, 0, 255, 0, 100);
    
    analogWrite(pins.ENA, speed);
    analogWrite(pins.ENB, speed);
}

// Robot directional control state machine
void directionSelect(int direction)
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

void setCurrentRobotDirection(int d)
{
    robotDirection = d;
}

int getCurrentRobotDirection()
{
    return robotDirection;
}

void moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(pins.IN1, HIGH);
    digitalWrite(pins.IN2, LOW);
    digitalWrite(pins.IN3, HIGH);
    digitalWrite(pins.IN4, LOW);
    setCurrentRobotDirection(forward);
}

void moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(pins.IN1, LOW);
    digitalWrite(pins.IN2, HIGH);
    digitalWrite(pins.IN3, LOW);
    digitalWrite(pins.IN4, HIGH);
    setCurrentRobotDirection(reverse);
}

void moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(pins.IN1, HIGH);
    digitalWrite(pins.IN2, LOW);
    digitalWrite(pins.IN3, LOW);
    digitalWrite(pins.IN4, LOW);
    setCurrentRobotDirection(right);
}
void moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(pins.IN1, LOW);
    digitalWrite(pins.IN2, LOW);
    digitalWrite(pins.IN3, HIGH);
    digitalWrite(pins.IN4, LOW);
    setCurrentRobotDirection(left);
}
void stopMove()
{
    Serial.println("STOP");
    digitalWrite(pins.IN1, LOW);
    digitalWrite(pins.IN2, LOW);
    digitalWrite(pins.IN3, LOW);
    digitalWrite(pins.IN4, LOW);
    setCurrentRobotDirection(stop);
}

// // Robot directional control state machine
// void DirectionControl::directionSelect(int direction)
// {
//     switch (direction)
//     {
//     case forward:
//         moveForward();
//         break;
//     case reverse:
//         moveBackwards();
//         break;

//     case right:
//         moveRight();
//         break;

//     case left:
//         moveLeft();
//         break;

//     case stop:
//         stopMove();
//         break;
//     }
// }

// void DirectionControl::setCurrentRobotDirection(int d)
// {
//     robotDirection = d;
// }

// int DirectionControl::getCurrentRobotDirection()
// {
//     return robotDirection;
// }

// void DirectionControl::moveForward()
// {
//     Serial.println("FORWARD");
//     digitalWrite(pins.IN1, HIGH);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, HIGH);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(forward);
// }

// void DirectionControl::moveBackwards()
// {
//     Serial.println("REVERSE");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, HIGH);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, HIGH);
//     setCurrentRobotDirection(reverse);
// }

// void DirectionControl::moveRight()
// {
//     Serial.println("RIGHT");
//     digitalWrite(pins.IN1, HIGH);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(right);
// }
// void DirectionControl::moveLeft()
// {
//     Serial.println("LEFT");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, HIGH);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(left);
// }
// void DirectionControl::stopMove()
// {
//     Serial.println("STOP");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(stop);
// }

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
#include "NeuralNetwork.h"

#include <math.h>

SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

int distanceArray[3];

volatile unsigned long currentTime = 0;
volatile unsigned long previousTime = 0;
int z = 0;

const int loopDelay = 500;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);

  direcetionSetup();
  speedControl(255/2);
  
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();

  NeuralNetworkSetup();
  train_nn();
}

void loop()
{
  currentTime = millis();
  if (currentTime - previousTime >= loopDelay)
  {

    previousTime = currentTime;

    sensorServo.ServoMovementRoutine(z);
    distanceArray[z] = ultraSonic.measureDistance();
    z++;
    if (sensorServo.getIsServoMovementComplete() == true)
    {
      z = 0;
      drive_nn(distanceArray);
      sensorServo.isServoMovementComplete(false);
    }
  }
}

