#ifndef SensorDistance_H
#define SensorDistance_H

#include "Pins.h"

class SensorDistance{
   private:
   int duration = 0;
   int distance = 0;
    public:
    int measureDistance();
    void ultraSonicSetup();

};
#endif