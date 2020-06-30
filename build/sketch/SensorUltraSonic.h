#ifndef SensorUltraSonic_H
#define SensorUltraSonic_H
#include <Arduino.h>
//#include "Pins.h"

class SensorUltraSonic
{
private:
    int duration = 0;
    int distance = 0;
    int count = 0;
    int clearCount = 0;
    int initialCondition = 500;

    const int measurementInterval = 250;

    unsigned long update = 0;

    const int echo = A5;
    const int trig = A4;

    void setDistance(int);
    void setIsClearCount(int);

public:
    void measureDistance();
    void ultraSonicSetup();

    int getDistance();
    int getIsClearCount();

    bool isClear();
};
#endif