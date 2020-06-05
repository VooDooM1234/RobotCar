#include "DirectionControl.h"
#include "Pins.h"

Pins motorPins;
// move commands wont work with motorPins.IN# for some reason
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;

void DirectionControl::direcetionSetup()
{
    Serial.println("Motor setup starting");
    pinMode(motorPins.IN1, OUTPUT);
    pinMode(motorPins.IN2, OUTPUT);
    pinMode(motorPins.IN3, OUTPUT);
    pinMode(motorPins.IN4, OUTPUT);
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
