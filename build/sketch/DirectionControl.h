#ifndef DirectionControl_H
#define DirectionControl_H

#include "Pins.h"

class DirectionControl
{
private:
    //motor control pins for l292n
    const int IN1 = 8;
    const int IN2 = 9;
    const int IN3 = 10;
    const int IN4 = 11;
    //speed control pins
    const int ENA = 5;
    const int ENB = 6;

    int robotDirection = 0;

    int robotSpeed = 125;

    const int robotMovementInterval = 1000;
    unsigned long currentTimeRobot = 0;
    unsigned long previousTimeRobot = 0;

    void moveForward();
    void moveBackwards();
    void moveLeft();
    void moveRight();
    void stopMove();
    
    void setCurrentRobotDirection(int);

public:
    void directionSelect(int direction);
    void direcetionSetup();

    int getCurrentRobotDirection();
    int getRobotSpeed();
    
    void setRobotSpeed(int);
    void speedControl();

    enum direction
    {
        forward = 1,
        reverse = 2,
        right = 3,
        left = 4,
        stop = 0
    };
};
#endif