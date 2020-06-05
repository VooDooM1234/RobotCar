#include <Arduino.h>

#include "SensorDistance.h"
#include "Pins.h"

Pins ultraSonicPins;

void SensorDistance::ultraSonicSetup()
{
    pinMode(ultraSonicPins.trig, OUTPUT);
    pinMode(ultraSonicPins.echo, INPUT);
}

int SensorDistance::measureDistance()
{
 digitalWrite(ultraSonicPins.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultraSonicPins.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraSonicPins.trig, LOW);

  duration = pulseIn(ultraSonicPins.echo, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}
