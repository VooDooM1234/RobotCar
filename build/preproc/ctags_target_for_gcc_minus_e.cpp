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

// Robot directional control state machine
void DirectionControl::directionSelect(int direction)
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


# 14 "e:\\Projects\\RobotCar\\in\\Main.ino" 2


# 15 "e:\\Projects\\RobotCar\\in\\Main.ino"
DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins = Pins();

int distanceArray[3];

volatile unsigned long currentTime = 0;
volatile unsigned long previousTime = 0;

const int loopDelay = 500;
volatile int i = 0;
int motorUpdate = 0;
bool goFlag = true;
// http://robotics.hobbizine.com/arduinoann.html

/******************************************************************

 * Network Configuration - customized per network 

 ******************************************************************/
# 36 "e:\\Projects\\RobotCar\\in\\Main.ino"
const int PatternCount = 8;
const int InputNodes = 3;
const int HiddenNodes = InputNodes + 1;
const int OutputNodes = 2;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

// const byte Input[PatternCount][InputNodes] = {
//     //LEFT, MIDDLE, RIGHT

//     {0, 0, 0}, //clear
//     {0, 0, 1}, //Right
//     {0, 1, 0}, //Middle
//     {1, 0, 0}, //Left
//     {0, 1, 1}, //Right and Middle
//     {1, 0, 1}, //Left and Right
//     {1, 1, 0}, //Left and Middle
//     {1, 1, 1}, //Left, Middle and Right
// };

// const byte Target[PatternCount][OutputNodes] = {
//     {directionControl.moveForward},
//     {directionControl.moveLeft},
//     {directionControl.moveBackwards},
//     {directionControl.moveRight},
//     {directionControl.moveLeft},
//     {directionControl.moveForward},
//     {directionControl.moveRight},
//     {directionControl.moveBackwards}};

// /******************************************************************
//  * End Network Configuration
//  ******************************************************************/

// int i, j, p, q, r;
// int ReportEvery1000;
// int RandomizedIndex[PatternCount];
// long TrainingCycle;
// float Rando;
// float Error;
// float Accum;

// float Hidden[HiddenNodes];
// float Output[OutputNodes];
// float HiddenWeights[InputNodes + 1][HiddenNodes];
// float OutputWeights[HiddenNodes + 1][OutputNodes];
// float HiddenDelta[HiddenNodes];
// float OutputDelta[OutputNodes];
// float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
// float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);
  // randomSeed(analogRead(3));
  // ReportEvery1000 = 1;
  // for (p = 0; p < PatternCount; p++)
  // {
  //   RandomizedIndex[p] = p;
  // }
  directionControl.direcetionSetup();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();
  directionControl.directionSelect(directionControl.direction::stop);

  // train_nn();
}

void loop()
{
  currentTime = millis();
  if (currentTime - previousTime >= loopDelay)
  {

    previousTime = currentTime;

    sensorServo.ServoMovementRoutine(i);
    distanceArray[i] = ultraSonic.measureDistance();
    Serial.println(distanceArray[i]);
    i++;
    if (sensorServo.getIsServoMovementComplete() == true)
    {
      i = 0;
      sensorServo.isServoMovementComplete(false);
    }

  }

  // if (sensorServo.getCurrentServoState() == sensorServo.left && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::left)
  // {
  //   directionControl.directionSelect(directionControl.direction::left);
  // }
  // else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == true && directionControl.getCurrentRobotDirection() != directionControl.direction::forward)
  // {
  //   directionControl.directionSelect(directionControl.direction::forward);
  // }
  // else if (sensorServo.getCurrentServoState() == sensorServo.right && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::right)
  // {
  //   directionControl.directionSelect(directionControl.direction::right);
  // }
  // else if (sensorServo.getCurrentServoState() == sensorServo.centre && ultraSonic.isClear() == false && directionControl.getCurrentRobotDirection() != directionControl.direction::reverse)
  // {
  //   directionControl.directionSelect(directionControl.direction::reverse);
  // }
}

// void train_nn() {
//   /******************************************************************
//     Initialize HiddenWeights and ChangeHiddenWeights
//   ******************************************************************/
//   prog_start = 0;
//   digitalWrite(LEDYEL, LOW);
//   for ( i = 0 ; i < HiddenNodes ; i++ ) {
//     for ( j = 0 ; j <= InputNodes ; j++ ) {
//       ChangeHiddenWeights[j][i] = 0.0 ;
//       Rando = float(random(100)) / 100;
//       HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
//     }
//   }
//   digitalWrite(LEDYEL, HIGH);
//   /******************************************************************
//     Initialize OutputWeights and ChangeOutputWeights
//   ******************************************************************/
//   digitalWrite(LEDRED, LOW);
//   for ( i = 0 ; i < OutputNodes ; i ++ ) {
//     for ( j = 0 ; j <= HiddenNodes ; j++ ) {
//       ChangeOutputWeights[j][i] = 0.0 ;
//       Rando = float(random(100)) / 100;
//       OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
//     }
//   }
//   digitalWrite(LEDRED, HIGH);
//   //SerialUSB.println("Initial/Untrained Outputs: ");
//   //toTerminal();
//   /******************************************************************
//     Begin training
//   ******************************************************************/

//   for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

//     /******************************************************************
//       Randomize order of training patterns
//     ******************************************************************/

//     for ( p = 0 ; p < PatternCount ; p++) {
//       q = random(PatternCount);
//       r = RandomizedIndex[p] ;
//       RandomizedIndex[p] = RandomizedIndex[q] ;
//       RandomizedIndex[q] = r ;
//     }
//     Error = 0.0 ;
//     /******************************************************************
//       Cycle through each training pattern in the randomized order
//     ******************************************************************/
//     for ( q = 0 ; q < PatternCount ; q++ ) {
//       p = RandomizedIndex[q];

//       /******************************************************************
//         Compute hidden layer activations
//       ******************************************************************/
//       digitalWrite(LEDYEL, LOW);
//       for ( i = 0 ; i < HiddenNodes ; i++ ) {
//         Accum = HiddenWeights[InputNodes][i] ;
//         for ( j = 0 ; j < InputNodes ; j++ ) {
//           Accum += Input[p][j] * HiddenWeights[j][i] ;
//         }
//         Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;
//       }
//       digitalWrite(LEDYEL, HIGH);

//       /******************************************************************
//         Compute output layer activations and calculate errors
//       ******************************************************************/
//       digitalWrite(LEDRED, LOW);
//       for ( i = 0 ; i < OutputNodes ; i++ ) {
//         Accum = OutputWeights[HiddenNodes][i] ;
//         for ( j = 0 ; j < HiddenNodes ; j++ ) {
//           Accum += Hidden[j] * OutputWeights[j][i] ;
//         }
//         Output[i] = 1.0 / (1.0 + exp(-Accum)) ;
//         OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
//         Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;
//       }
//       //SerialUSB.println(Output[0]*100);
//       digitalWrite(LEDRED, HIGH);
//       /******************************************************************
//         Backpropagate errors to hidden layer
//       ******************************************************************/
//       digitalWrite(LEDYEL, LOW);
//       for ( i = 0 ; i < HiddenNodes ; i++ ) {
//         Accum = 0.0 ;
//         for ( j = 0 ; j < OutputNodes ; j++ ) {
//           Accum += OutputWeights[i][j] * OutputDelta[j] ;
//         }
//         HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
//       }
//       digitalWrite(LEDYEL, HIGH);

//       /******************************************************************
//         Update Inner-->Hidden Weights
//       ******************************************************************/

//       digitalWrite(LEDRED, LOW);
//       for ( i = 0 ; i < HiddenNodes ; i++ ) {
//         ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;
//         HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;
//         for ( j = 0 ; j < InputNodes ; j++ ) {
//           ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
//           HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
//         }
//       }
//       digitalWrite(LEDRED, HIGH);
//       /******************************************************************
//         Update Hidden-->Output Weights
//       ******************************************************************/
//       digitalWrite(LEDYEL, LOW);
//       for ( i = 0 ; i < OutputNodes ; i ++ ) {
//         ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;
//         OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;
//         for ( j = 0 ; j < HiddenNodes ; j++ ) {
//           ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
//           OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
//         }
//       }
//       digitalWrite(LEDYEL, HIGH);
//     }

//     /******************************************************************
//       Every 100 cycles send data to terminal for display and draws the graph on OLED
//     ******************************************************************/
//     ReportEvery1000 = ReportEvery1000 - 1;
//     if (ReportEvery1000 == 0)
//     {
//       int graphNum = TrainingCycle / 100;
//       int graphE1 = Error * 1000;
//       int graphE = map(graphE1, 3, 80, 47, 0);
//       ErrorGraph[graphNum] = graphE;
//       u8g2.firstPage();
//       do {
//         drawGraph();
//       } while ( u8g2.nextPage() );

//       SerialUSB.println();
//       SerialUSB.println();
//       SerialUSB.print ("TrainingCycle: ");
//       SerialUSB.print (TrainingCycle);
//       SerialUSB.print ("  Error = ");
//       SerialUSB.println (Error, 5);
//       SerialUSB.print ("  Graph Num: ");
//       SerialUSB.print (graphNum);
//       SerialUSB.print ("  Graph Error1 = ");
//       SerialUSB.print (graphE1);
//       SerialUSB.print ("  Graph Error = ");
//       SerialUSB.println (graphE);

//       toTerminal();

//       if (TrainingCycle == 1)
//       {
//         ReportEvery1000 = 99;
//       }
//       else
//       {
//         ReportEvery1000 = 100;
//       }
//     }

//     /******************************************************************
//       If error rate is less than pre-determined threshold then end
//     ******************************************************************/

//     if ( Error < Success ) break ;
//   }
// }

// void drive_nn()
// {
//   SerialUSB.println("Running NN Drive Test");
//   if (Success < Error) {
//     prog_start = 0;
//     SerialUSB.println("NN not Trained");
//   }
//   while (Error < Success) {
//     int num;
//     int farDist = 35;
//     int closeDist = 7;
//     float TestInput[] = {0, 0, 0};

//     int Left = analogRead(A1);   // Collect sonar distances.
//     int Middle = analogRead(A2);   // Collect sonar distances.
//     int Right = analogRead(A3);   // Collect sonar distances.

//     LL1 = map(LL1, 400, 1024, 0, 100);
//     LL2 = map(LL2, 400, 1024, 0, 100);
//     LL3 = map(LL3, 400, 1024, 0, 100);
//     LL4 = map(LL4, 400, 1024, 0, 100);

//     LL1 = constrain(LL1, 0, 100);
//     LL2 = constrain(LL2, 0, 100);
//     LL3 = constrain(LL3, 0, 100);
//     LL4 = constrain(LL4, 0, 100);

//     TestInput[0] = float(LL1) / 100;
//     TestInput[1] = float(LL2) / 100;
//     TestInput[2] = float(LL3) / 100;
//     TestInput[3] = float(LL4) / 100;
// #ifdef DEBUG
//     SerialUSB.print("Input: ");
//     SerialUSB.print(TestInput[3], 2);
//     SerialUSB.print("\t");
//     SerialUSB.print(TestInput[2], 2);
//     SerialUSB.print("\t");
//     SerialUSB.print(TestInput[1], 2);
//     SerialUSB.print("\t");
//     SerialUSB.println(TestInput[0], 2);
// #endif

//     InputToOutput(TestInput[0], TestInput[1], TestInput[2], TestInput[3]); //INPUT to ANN to obtain OUTPUT

//     int speedA = Output[0] * 100;
//     int speedB = Output[1] * 100;
//     speedA = int(speedA);
//     speedB = int(speedB);
// #ifdef DEBUG
//     SerialUSB.print("Speed: ");
//     SerialUSB.print(speedA);
//     SerialUSB.print("\t");
//     SerialUSB.println(speedB);
// #endif
//     motorA(speedA);
//     motorB(speedB);
//     delay(50);
//   }
// }

// void toTerminal()
// {

//   for (p = 0; p < PatternCount; p++)
//   {
//     Serial.println();
//     Serial.print("  Training Pattern: ");
//     Serial.println(p);
//     Serial.print("  Input ");
//     for (i = 0; i < InputNodes; i++)
//     {
//       Serial.print(Input[p][i], DEC);
//       Serial.print(" ");
//     }
//     Serial.print("  Target ");
//     for (i = 0; i < OutputNodes; i++)
//     {
//       Serial.print(Target[p][i], DEC);
//       Serial.print(" ");
//     }
//     /******************************************************************
// * Compute hidden layer activations
// ******************************************************************/

//     for (i = 0; i < HiddenNodes; i++)
//     {
//       Accum = HiddenWeights[InputNodes][i];
//       for (j = 0; j < InputNodes; j++)
//       {
//         Accum += Input[p][j] * HiddenWeights[j][i];
//       }
//       Hidden[i] = 1.0 / (1.0 + exp(-Accum));
//     }

//     /******************************************************************
// * Compute output layer activations and calculate errors
// ******************************************************************/

//     for (i = 0; i < OutputNodes; i++)
//     {
//       Accum = OutputWeights[HiddenNodes][i];
//       for (j = 0; j < HiddenNodes; j++)
//       {
//         Accum += Hidden[j] * OutputWeights[j][i];
//       }
//       Output[i] = 1.0 / (1.0 + exp(-Accum));
//     }
//     Serial.print("  Output ");
//     for (i = 0; i < OutputNodes; i++)
//     {
//       Serial.print(Output[i], 5);
//       Serial.print(" ");
//     }
//   }
// }
