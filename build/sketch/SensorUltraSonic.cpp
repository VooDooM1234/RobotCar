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
  Serial.print("Get Distance: ");
  Serial.println(distance);
  return distance;
}

void SensorUltraSonic::ultraSonicSetup()
{
  Serial.println("Ultrasonic Setup");
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

int SensorUltraSonic::measureDistance( )
{
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
  return distance;

}

bool SensorUltraSonic::isClear()
{
  if (this->distance >= 15)
  {
    // Serial.println("Returning Clear");
    return true;
  }
  else
  {
    // Serial.println("Returning object detected");
    return false;
  }
}
