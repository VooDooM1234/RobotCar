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

DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins;

const int stopMovingInterval = 500;
int motorUpdate = 0;
bool goFlag = true;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  directionControl.direcetionSetup();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();
  directionControl.directionSelect(directionControl.direction::stop);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance();

  if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::left)
  {
    directionControl.directionSelect(directionControl.direction::left);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true && directionControl.getCurrentRobotDirection() != directionControl.direction::forward)
  {
    directionControl.directionSelect(directionControl.direction::forward);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::right)
  {
    directionControl.directionSelect(directionControl.direction::right);
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::reverse)
  {
    directionControl.directionSelect(directionControl.direction::reverse);
  }
  

 
}
