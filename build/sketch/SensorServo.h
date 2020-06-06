#ifndef SensorServo_H
#define SensorServo_H

#include "Pins.h"

class SensorServo
{
private:
    const int sonarServoPin = 12;
    const int servoMoveInterval = 1000;
   

    long lastUpdate = 0;
    

    bool motorTick = false;

    void servoMoveLeft();
    void servoMoveRight();
    void servoMoveCentre();
    void directionSelect(int direction);

public:
    void sensorServoSetup();
    void ServoMovementRoutine();
    void isServoMovementComplete(bool);

    bool getIsServoMovementComplete();
};
#endif