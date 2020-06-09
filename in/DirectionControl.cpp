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
