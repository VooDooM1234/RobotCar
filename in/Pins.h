#ifndef Pins_H
#define Pins_H

typedef struct Pins
{
  Pins();
  int IN1;
  int IN2;
  int IN3;
  int IN4;

  int ENA = 5;
  int ENB = 6;

  int echo;
  int trig;

  int sonarMotorPin;
} Pins;

extern Pins pins;
#endif
