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
void SensorServo::ServoMovementRoutine()
{
    switch (servoState)
    {
    case 1:
        currentTime = millis();
        if (currentTime - previousTime >= servoMoveInterval)
        {
            servoMoveCentre();
            previousTime = currentTime;
            servoState = 2;
        }
        break;
    case 2:
        currentTime = millis();
        if (currentTime - previousTime >= servoMoveInterval)
        {
            servoMoveLeft();
            previousTime = currentTime;
            servoState = 3;
        }
        break;
    case 3:
        currentTime = millis();
        if (currentTime - previousTime >= servoMoveInterval)
        {
            servoMoveRight();
            previousTime = currentTime;
            servoState = 1;
        }
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
    sonarServo.write(0);
    setCurrentServoState(left);
    Serial.println("Sonar moving left");
}
void SensorServo::servoMoveRight()
{
    sonarServo.write(180);
    setCurrentServoState(right);
    Serial.println("Sonar moving right");
}
void SensorServo::servoMoveCentre()
{
    sonarServo.write(90);
    setCurrentServoState(centre);
    Serial.println("Sonar moving centre");
}