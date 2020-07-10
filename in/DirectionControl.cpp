#include "DirectionControl.h"
#include "Pins.h"

Pins pins;

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
