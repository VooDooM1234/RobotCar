#ifndef SensorUltraSonic_H
#define SensorUltraSonic_H
#include <Arduino.h>
//#include "Pins.h"

bool isClear(int);

class SensorUltraSonic
{
    
private:
    int duration = 0;
    int distance = 0;
    int count = 0;
    int initialCondition = 1000;
    const int measurementInterval = 500;

    long update = 0;

    const int echo = A5;
    const int trig = A4;

public:
    int measureDistance();
    void ultraSonicSetup();
    void setDistance(int);

    int getDistance();

    
};
#endif