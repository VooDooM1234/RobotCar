#ifndef DirectionControl_H
#define DirectionControl_H

#include "Pins.h"

class DirectionControl{
    private:
   void moveForward();
    void moveBackwards();
    void moveLeft();
    void moveRight();
    void stopMove();

    const int speed = 255;
    public:
    void directionSelect(int direction);
    void direcetionSetup();
   
    enum direction{forward = 1, reverse = 2, right = 3, left= 4, stop= 0};

};
#endif