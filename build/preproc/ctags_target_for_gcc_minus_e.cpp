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
    Serial.println("Motor setup complete");
}
void DirectionControl::directionSelect(int direction){
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


    default:
    stopMove();
        break;

    }
}

void DirectionControl::moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(IN1, 0x1);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x1);
    digitalWrite(IN4, 0x0);
}

void DirectionControl::moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x1);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x1);
}

void DirectionControl::moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(IN1, 0x1);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x0);
}
void DirectionControl::moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x1);
    digitalWrite(IN4, 0x0);
}
void DirectionControl::stopMove()
{
    Serial.println("STOP");
    digitalWrite(IN1, 0x0);
    digitalWrite(IN2, 0x0);
    digitalWrite(IN3, 0x0);
    digitalWrite(IN4, 0x0);
}
# 1 "e:\\Projects\\RobotCar\\in\\Main.ino"
/*

    AVR Robot car code

*/
# 5 "e:\\Projects\\RobotCar\\in\\Main.ino"
# 6 "e:\\Projects\\RobotCar\\in\\Main.ino" 2
# 7 "e:\\Projects\\RobotCar\\in\\Main.ino" 2


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
  directionControl.directionSelect(directionControl.direction::forward);
}

void loop()
{
  sensorServo.ServoMovementRoutine();
  ultraSonic.measureDistance(sensorServo.getIsServoMovementComplete());

  Serial.println("Is Clear?: ");
  Serial.println(ultraSonic.isClear(ultraSonic.getDistance()));

  Serial.print("Get Distance: ");
  Serial.println(ultraSonic.getDistance());

  if (ultraSonic.isClear(ultraSonic.getDistance()) == true && goFlag == true)
  {
    directionControl.directionSelect(directionControl.direction::forward);
  }
  else
  {
    directionControl.directionSelect(directionControl.direction::stop);
    goFlag = false;
  }

  if (millis() >= motorUpdate + stopMovingInterval)
  {
    motorUpdate += stopMovingInterval;
    goFlag = true;
  }
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}
