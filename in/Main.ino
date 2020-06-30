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

#define MAX_SPEED 255
#define HALF_SPEED (MAX_SPEED / 2)

DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins;

bool flag = false;
unsigned long previousTimeFlag = 0;
int flagInterval = 5000;
unsigned long currentTimeFlag = 0;

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  directionControl.direcetionSetup();
  directionControl.speedControl();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();
  directionControl.setRobotSpeed(255);
  directionControl.directionSelect(directionControl.direction::stop);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance();

  currentTimeFlag = millis();

  if (currentTimeFlag - previousTimeFlag >= flagInterval)
  {
    previousTimeFlag = millis();
    flag = true;
  }

  if (flag)
  {
    if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false)
    {
      //directionControl.setRobotSpeed(MAX_SPEED);
      directionControl.directionSelect(directionControl.direction::left);
    }
    else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false)

    {
      //directionControl.setRobotSpeed(MAX_SPEED);
      directionControl.directionSelect(directionControl.direction::right);
    }
    else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true)
    {

      // directionControl.setRobotSpeed(HALF_SPEED);
      directionControl.directionSelect(directionControl.direction::forward);
    }
    else if (/*ultraSonic.getDistance() <= 5*/ sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false)
    {
      //directionControl.setRobotSpeed(MAX_SPEED);
      directionControl.directionSelect(directionControl.direction::reverse);
    }
    flag = false;
  }
}
