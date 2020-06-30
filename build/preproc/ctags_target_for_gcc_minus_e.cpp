# 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
# 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
# 2 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp" 2


void DirectionControl::direcetionSetup()
{
    Serial.println("Motor setup starting");
    pinMode(IN1, 0x1);
    pinMode(IN2, 0x1);
    pinMode(IN3, 0x1);
    pinMode(IN4, 0x1);
    pinMode(ENA, 0x1);
    pinMode(ENB, 0x1);
    Serial.println("Motor setup complete");
}

// Robot directional control state machine
void DirectionControl::directionSelect(int direction)
{
    currentTimeRobot = millis();
    if (currentTimeRobot - previousTimeRobot >= robotMovementInterval)
    {
        switch (direction)
        {
        case forward:
            moveForward();
            break;
        case reverse:
            moveBackwards();
            break;

        case right:
            moveRight();
            break;

        case left:
            moveLeft();
            break;

        case stop:
            stopMove();
            break;
        }
        previousTimeRobot = currentTimeRobot;
    }
}
void DirectionControl::speedControl()
{
    analogWrite(ENA, robotSpeed);
    analogWrite(ENB, robotSpeed);
}

void DirectionControl::setRobotSpeed(int s)
{
    robotSpeed = s;
    speedControl();
}

int DirectionControl::getRobotSpeed()
{
    return robotSpeed;
}

void DirectionControl::setCurrentRobotDirection(int d)
{
    robotDirection = d;
}

int DirectionControl::getCurrentRobotDirection()
{
    return robotDirection;
}

void DirectionControl::moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(IN1, 0x1);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x1);
    digitalWrite(IN4, 0x0);
    setCurrentRobotDirection(forward);
}

void DirectionControl::moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x1);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x1);
    setCurrentRobotDirection(reverse);
}

void DirectionControl::moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(IN1, 0x1);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x0);
    setCurrentRobotDirection(right);
}
void DirectionControl::moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x1);
    digitalWrite(IN4, 0x0);
    setCurrentRobotDirection(left);
}
void DirectionControl::stopMove()
{
    Serial.println("STOP");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x0);
    setCurrentRobotDirection(stop);
}
# 1 "e:\\Projects\\RobotCar\\in\\Main.ino"
/**

 * @brief  Robot car with ultrasonic senor collision detection

 * @note   

 * @baudRate: 9600

 * @board: Arduino uno

 * @retval 

 */
# 9 "e:\\Projects\\RobotCar\\in\\Main.ino"
# 10 "e:\\Projects\\RobotCar\\in\\Main.ino" 2
# 11 "e:\\Projects\\RobotCar\\in\\Main.ino" 2





DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins;

bool flag = false;
unsigned long previousTimeFlag = 0;
int flagInterval = 1000;
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

  if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false && flag == true)
  {
    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::left);
    flag = false;
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false && flag == true)

  {

    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::right);
    flag = false;
  }
  else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true && flag == true)
  {

    // directionControl.setRobotSpeed(HALF_SPEED);
    directionControl.directionSelect(directionControl.direction::forward);
    flag = false;
  }
  else if (/*ultraSonic.getDistance() <= 5*/ sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false)
  {
    //directionControl.setRobotSpeed(MAX_SPEED);
    directionControl.directionSelect(directionControl.direction::reverse);
    flag = false;
  }

  if (currentTimeFlag - previousTimeFlag >= flagInterval)
  {
    flag = true;
  }
}
