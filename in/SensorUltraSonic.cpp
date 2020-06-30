#include <Arduino.h>

#include "SensorUltraSonic.h"
#include "Pins.h"

Pins ultraSonicPins;

void SensorUltraSonic::setDistance(int d)
{
  distance = d;
}

int SensorUltraSonic::getDistance()
{
 // Serial.print("Get Distance: ");
  Serial.println(distance);
  return distance;
}

void SensorUltraSonic::ultraSonicSetup()
{
  Serial.println("Ultrasonic Setup");
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void SensorUltraSonic::measureDistance()
{
  if (millis() - update >= measurementInterval && millis() >= initialCondition)
  {
    if (count != 0)
    {
      update += measurementInterval;
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);

      duration = pulseIn(echo, HIGH);
      distance = (duration * .0343) / 2;
      Serial.print("Distance (cm): ");
      Serial.println(distance);
      setDistance(distance);
      count = 0;
    }
    count = 1;
  }
}

void SensorUltraSonic::setIsClearCount(int c)
{
  clearCount = c;
}

int SensorUltraSonic::getIsClearCount()
{
  return clearCount;
}

bool SensorUltraSonic::isClear()
{
  if (this->distance >= 15)
  {
    clearCount++;
    return true;
  }
  else
  {
    clearCount--;
    return false;
  }
}
