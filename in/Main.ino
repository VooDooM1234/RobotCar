/**
 * @brief  Robot car with ultrasonic senor collision detection
 * @note   
 * @baudRate: 9600
 * @board: Arduino uno
 * @retval 
 */
#include "DirectionControl.h"
#include "SensorServo.h"
#include "SensorUltraSonic.h"
#include "Pins.h"
#include "NeuralNetwork.h"

#include <math.h>

SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

int distanceArray[3];

volatile unsigned long currentTime = 0;
volatile unsigned long previousTime = 0;
int z = 0;

const int loopDelay = 500;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);

  direcetionSetup();
  speedControl(255/2);
  
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();

  NeuralNetworkSetup();
  train_nn();
}

void loop()
{
  currentTime = millis();
  if (currentTime - previousTime >= loopDelay)
  {

    previousTime = currentTime;

    sensorServo.ServoMovementRoutine(z);
    distanceArray[z] = ultraSonic.measureDistance();
    z++;
    if (sensorServo.getIsServoMovementComplete() == true)
    {
      z = 0;
      drive_nn(distanceArray);
      sensorServo.isServoMovementComplete(false);
    }
  }
}
