/*
    AVR Robot car code
*/
#include "DirectionControl.h"
#include "Pins.h"
#include <Servo.h>

DirectionControl directionControl = DirectionControl();
Servo sonarServo;
Pins _pins;

int distance = 0;


void setup()
{
Serial.println("Lanuching Setup");
Serial.begin(9600);
directionControl.direcetionSetup();

sonarServo.attach(_pins.sonarMotorPin);
sonarServo.write(90);
}


void loop()
{
    
    
    delay(500);
    
    //directionControl.directionSelect(directionControl.direction::forward);
   // delay(1000);
   // directionControl.directionSelect(directionControl.direction::stop);
   // delay(1000);
 
}

void servoMoveLeft(){
  sonarServo.write(0);
  delay(500);
}


