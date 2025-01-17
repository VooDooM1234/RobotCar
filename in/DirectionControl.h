#ifndef DirectionControl_H
#define DirectionControl_H

#include "Pins.h"

void direcetionSetup();
void speedControl();

int robotDirection = 0;

void moveForward();
void moveBackwards();
void moveLeft();
void moveRight();
void stopMove();

void setCurrentRobotDirection(int);

const int speed = 255;

void directionSelect(int direction);

int getCurrentRobotDirection();

enum direction
{
    forward = 1,
    reverse = 2,
    right = 3,
    left = 4,
    stop = 0
};
// class DirectionControl
// {
// private:
//     // const int IN1 = 8;
//     // const int IN2 = 9;
//     // const int IN3 = 10;
//     // const int IN4 = 11;

//     int robotDirection = 0;

//     void moveForward();
//     void moveBackwards();
//     void moveLeft();
//     void moveRight();
//     void stopMove();

//     void setCurrentRobotDirection(int);

//     const int speed = 255;

// public:
//     void directionSelect(int direction);

//     int getCurrentRobotDirection();

//     enum direction
//     {
//         forward = 1,
//         reverse = 2,
//         right = 3,
//         left = 4,
//         stop = 0
//     };
// };
#endif