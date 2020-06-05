#include "Pins.h"
#include <Arduino.h>


Pins::Pins()
{
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;

int echo = A5;
int trig = A4;

int sonarMotorPin = 12;
}

Pins pins;
