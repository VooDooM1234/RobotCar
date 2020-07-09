#ifndef SensorServo_H
#define SensorServo_H

#include "Pins.h"

class SensorServo
{
private:
    const int sonarServoPin = 12;
    const int servoMoveInterval = 1000;

    volatile unsigned long currentTime = 0;
    volatile unsigned long previousTime = 0;

    //initial servo state set to centre postion.
    int currentServoState = 0;

    bool motorTick = false;

    void servoMoveLeft();
    void servoMoveRight();
    void servoMoveCentre();
    
    void setCurrentServoState(int);

public:
    int centre = 1;
    int left = 2;
    int right = 3;
    
    void sensorServoSetup();
    void ServoMovementRoutine(int);

    bool getIsServoMovementComplete();
void isServoMovementComplete(bool);
    int getCurrentServoState();
};
#endif