#ifndef SensorServo_H
#define SensorServo_H

#include "Pins.h"

class SensorServo
{
private:
    void servoMoveLeft();
    void servoMoveRight();
    void ServoMovecentre();

public:
    void sensorServoSetup();
    void ServoMovementRoutine();
};
#endif