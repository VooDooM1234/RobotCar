#ifndef SensorUltraSonic_H
#define SensorUltraSonic_H
#include <Arduino.h>
//#include "Pins.h"

class SensorUltraSonic
{
private:
    int duration = 0;
    int distance = 10;
    const int measurementInterval = 500;

    long update = 0;

    const int echo = A5;
    const int trig = A4;

public:
    void measureDistance(bool);
    void ultraSonicSetup();
    void setDistance(int);

    int getDistance();

    bool isClear(int);
};
#endif