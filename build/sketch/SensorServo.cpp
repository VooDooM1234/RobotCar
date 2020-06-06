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

void SensorServo::ServoMovementRoutine()
{
    int i = 1;
    while (i <= 3)
    {
        //trigger servo movement at every 1000ms interval
        if (millis() >= lastUpdate + servoMoveInterval)
        {
            lastUpdate += servoMoveInterval;

            directionSelect(i);
            // Serial.print("Servo Movement Time:");
            // Serial.println(lastUpdate);
            motorTick = true;
           
            i++;
            isServoMovementComplete(true);
        }
        isServoMovementComplete(false);
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

void SensorServo::directionSelect(int direction)
{
    switch (direction)
    {
    case 1:
        servoMoveCentre();
        break;
    case 2:
        servoMoveRight();
        break;
    case 3:
        servoMoveLeft();
        break;
    }
}

void SensorServo::servoMoveLeft()
{
    sonarServo.write(0);
    Serial.println("Sonar moving left");
}
void SensorServo::servoMoveRight()
{
    sonarServo.write(180);
    Serial.println("Sonar moving right");
}
void SensorServo::servoMoveCentre()
{
    sonarServo.write(90);
    Serial.println("Sonar moving centre");
}