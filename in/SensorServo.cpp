#include "SensorServo.h"
#include "SensorUltraSonic.h"

#include <Arduino.h>
#include <Servo.h>

Servo sonarServo;

void SensorServo::sensorServoSetup()
{
    Serial.println("Servo setup");
    servoMoveCentre();
    sonarServo.attach(sonarServoPin);
}
//Servo stat Machine Service routine
//Switches every interval of 1000ms
void SensorServo::ServoMovementRoutine(int servoState)
{
    switch (servoState)
    {
    case 0:

        servoMoveLeft();

        break;
    case 1:
        servoMoveCentre();

        break;
    case 2:

        servoMoveRight();
        isServoMovementComplete(true);
        break;
    }
}
void SensorServo::isServoMovementComplete(bool m)
{
    motorTick = m;
}

bool SensorServo::getIsServoMovementComplete()
{
    return motorTick;
}

void SensorServo::setCurrentServoState(int state)
{
    currentServoState = state;
}

int SensorServo::getCurrentServoState()
{
    return currentServoState;
}

void SensorServo::servoMoveLeft()
{
    sonarServo.write(45);
    setCurrentServoState(left);
    Serial.println("Sonar moving left");
}
void SensorServo::servoMoveRight()
{
    sonarServo.write(135);
    setCurrentServoState(right);
    Serial.println("Sonar moving right");
}
void SensorServo::servoMoveCentre()
{
    sonarServo.write(90);
    setCurrentServoState(centre);
    Serial.println("Sonar moving centre");
}