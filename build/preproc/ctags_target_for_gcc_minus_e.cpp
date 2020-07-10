# 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
# 1 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp"
# 2 "e:\\Projects\\RobotCar\\in\\DirectionControl.cpp" 2


Pins pins;

void direcetionSetup()
{
    Serial.println("Motor setup starting");
    pinMode(pins.IN1, 0x1);
    pinMode(pins.IN2, 0x1);
    pinMode(pins.IN3, 0x1);
    pinMode(pins.IN4, 0x1);
    Serial.println("Motor setup complete");

    pinMode(pins.ENA, 0x1);
    pinMode(pins.ENB, 0x1);
}

void speedControl(int speed)
{
    speed = map(speed, 0, 255, 0, 100);

    analogWrite(pins.ENA, speed);
    analogWrite(pins.ENB, speed);
}

// Robot directional control state machine
void directionSelect(int direction)
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
}

void setCurrentRobotDirection(int d)
{
    robotDirection = d;
}

int getCurrentRobotDirection()
{
    return robotDirection;
}

void moveForward()
{
    Serial.println("FORWARD");
    digitalWrite(pins.IN1, 0x1);
    digitalWrite(pins.IN2, 0x0);
    digitalWrite(pins.IN3, 0x1);
    digitalWrite(pins.IN4, 0x0);
    setCurrentRobotDirection(forward);
}

void moveBackwards()
{
    Serial.println("REVERSE");
    digitalWrite(pins.IN1, 0x0);
    digitalWrite(pins.IN2, 0x1);
    digitalWrite(pins.IN3, 0x0);
    digitalWrite(pins.IN4, 0x1);
    setCurrentRobotDirection(reverse);
}

void moveRight()
{
    Serial.println("RIGHT");
    digitalWrite(pins.IN1, 0x1);
    digitalWrite(pins.IN2, 0x0);
    digitalWrite(pins.IN3, 0x0);
    digitalWrite(pins.IN4, 0x0);
    setCurrentRobotDirection(right);
}
void moveLeft()
{
    Serial.println("LEFT");
    digitalWrite(pins.IN1, 0x0);
    digitalWrite(pins.IN2, 0x0);
    digitalWrite(pins.IN3, 0x1);
    digitalWrite(pins.IN4, 0x0);
    setCurrentRobotDirection(left);
}
void stopMove()
{
    Serial.println("STOP");
    digitalWrite(pins.IN1, 0x0);
    digitalWrite(pins.IN2, 0x0);
    digitalWrite(pins.IN3, 0x0);
    digitalWrite(pins.IN4, 0x0);
    setCurrentRobotDirection(stop);
}

// // Robot directional control state machine
// void DirectionControl::directionSelect(int direction)
// {
//     switch (direction)
//     {
//     case forward:
//         moveForward();
//         break;
//     case reverse:
//         moveBackwards();
//         break;

//     case right:
//         moveRight();
//         break;

//     case left:
//         moveLeft();
//         break;

//     case stop:
//         stopMove();
//         break;
//     }
// }

// void DirectionControl::setCurrentRobotDirection(int d)
// {
//     robotDirection = d;
// }

// int DirectionControl::getCurrentRobotDirection()
// {
//     return robotDirection;
// }

// void DirectionControl::moveForward()
// {
//     Serial.println("FORWARD");
//     digitalWrite(pins.IN1, HIGH);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, HIGH);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(forward);
// }

// void DirectionControl::moveBackwards()
// {
//     Serial.println("REVERSE");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, HIGH);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, HIGH);
//     setCurrentRobotDirection(reverse);
// }

// void DirectionControl::moveRight()
// {
//     Serial.println("RIGHT");
//     digitalWrite(pins.IN1, HIGH);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(right);
// }
// void DirectionControl::moveLeft()
// {
//     Serial.println("LEFT");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, HIGH);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(left);
// }
// void DirectionControl::stopMove()
// {
//     Serial.println("STOP");
//     digitalWrite(pins.IN1, LOW);
//     digitalWrite(pins.IN2, LOW);
//     digitalWrite(pins.IN3, LOW);
//     digitalWrite(pins.IN4, LOW);
//     setCurrentRobotDirection(stop);
// }
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

# 13 "e:\\Projects\\RobotCar\\in\\Main.ino" 2

# 15 "e:\\Projects\\RobotCar\\in\\Main.ino" 2


# 16 "e:\\Projects\\RobotCar\\in\\Main.ino"
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
