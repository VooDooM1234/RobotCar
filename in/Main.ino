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

#include <math.h>

#define SONAR_LEFT 0
#define SONAR_MIDDLE 1
#define SONAR_RIGHT 2

DirectionControl directionControl = DirectionControl();
SensorServo sensorServo = SensorServo();
SensorUltraSonic ultraSonic = SensorUltraSonic();

Pins pins = Pins();

int distanceArray[3];

volatile unsigned long currentTime = 0;
volatile unsigned long previousTime = 0;
volatile int z = 0;

const int ENA = 5;
const int ENB = 6;

const int loopDelay = 500;


// http://robotics.hobbizine.com/arduinoann.html

/*************************
 * Network Configuration *
 **************************/

const int PatternCount = 8;
const int InputNodes = 3;
const int HiddenNodes = InputNodes + 1;
const int OutputNodes = 2;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

//Input Nodes
const byte Input[PatternCount][InputNodes] = {
    //LEFT, MIDDLE, RIGHT

    {0, 0, 0}, //clear
    {0, 0, 1}, //Right
    {0, 1, 0}, //Middle
    {1, 0, 0}, //Left
    {0, 1, 1}, //Right and Middle
    {1, 0, 1}, //Left and Right
    {1, 1, 0}, //Left and Middle
    {1, 1, 1}, //Left, Middle and Right
};

//Target Nodes
const byte Target[PatternCount][OutputNodes] = {
    {1, 1},
    {1, 0},
    {0, 0},
    {1, 0},
    {0, 1},
    {1, 1},
    {0, 1},
    {0, 0}};

/******************************************************************
 * End Network Configuration
 ******************************************************************/

int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long TrainingCycle;
float Rando;
float Error;
float Accum;

float Hidden[HiddenNodes];
float Output[OutputNodes];
float HiddenWeights[InputNodes + 1][HiddenNodes];
float OutputWeights[HiddenNodes + 1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

void setup()
{
  Serial.println("Lanuching Setup");
  Serial.begin(9600);

  directionControl.direcetionSetup();
  sensorServo.sensorServoSetup();
  ultraSonic.ultraSonicSetup();


  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  // Network Setup
  randomSeed(analogRead(3));
  ReportEvery1000 = 1;
  for (p = 0; p < PatternCount; p++)
  {
    RandomizedIndex[p] = p;
  }

  Serial.println("training nerual network");
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
      drive_nn();
      sensorServo.isServoMovementComplete(false);
    }
  }
}

void train_nn()
{
  /******************************************************************
    Initialize HiddenWeights and ChangeHiddenWeights
  ******************************************************************/

  for (i = 0; i < HiddenNodes; i++)
  {
    for (j = 0; j <= InputNodes; j++)
    {
      ChangeHiddenWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      HiddenWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }

  /******************************************************************
    Initialize OutputWeights and ChangeOutputWeights
  ******************************************************************/

  for (i = 0; i < OutputNodes; i++)
  {
    for (j = 0; j <= HiddenNodes; j++)
    {
      ChangeOutputWeights[j][i] = 0.0;
      Rando = float(random(100)) / 100;
      OutputWeights[j][i] = 2.0 * (Rando - 0.5) * InitialWeightMax;
    }
  }

  //Serial.println("Initial/Untrained Outputs: ");
  //toTerminal();
  /******************************************************************
    Begin training
  ******************************************************************/

  for (TrainingCycle = 1; TrainingCycle < 2147483647; TrainingCycle++)
  {

    /******************************************************************
      Randomize order of training patterns
    ******************************************************************/

    for (p = 0; p < PatternCount; p++)
    {
      q = random(PatternCount);
      r = RandomizedIndex[p];
      RandomizedIndex[p] = RandomizedIndex[q];
      RandomizedIndex[q] = r;
    }
    Error = 0.0;
    /******************************************************************
      Cycle through each training pattern in the randomized order
    ******************************************************************/
    for (q = 0; q < PatternCount; q++)
    {
      p = RandomizedIndex[q];

      /******************************************************************
        Compute hidden layer activations
      ******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        Accum = HiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++)
        {
          Accum += Input[p][j] * HiddenWeights[j][i];
        }
        Hidden[i] = 1.0 / (1.0 + exp(-Accum));
      }

      /******************************************************************
        Compute output layer activations and calculate errors
      ******************************************************************/

      for (i = 0; i < OutputNodes; i++)
      {
        Accum = OutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++)
        {
          Accum += Hidden[j] * OutputWeights[j][i];
        }
        Output[i] = 1.0 / (1.0 + exp(-Accum));
        OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]);
        Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]);
      }
      //Serial.println(Output[0]*100);

      /******************************************************************
        Backpropagate errors to hidden layer
      ******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        Accum = 0.0;
        for (j = 0; j < OutputNodes; j++)
        {
          Accum += OutputWeights[i][j] * OutputDelta[j];
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]);
      }

      /******************************************************************
        Update Inner-->Hidden Weights
      ******************************************************************/

      for (i = 0; i < HiddenNodes; i++)
      {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i];
        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i];
        for (j = 0; j < InputNodes; j++)
        {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];
          HiddenWeights[j][i] += ChangeHiddenWeights[j][i];
        }
      }

      /******************************************************************
        Update Hidden-->Output Weights
      ******************************************************************/

      for (i = 0; i < OutputNodes; i++)
      {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i];
        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i];
        for (j = 0; j < HiddenNodes; j++)
        {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
          OutputWeights[j][i] += ChangeOutputWeights[j][i];
        }
      }
    }

    /******************************************************************
      Every 100 cycles send data to terminal for display and draws the graph on OLED
    ******************************************************************/
    ReportEvery1000 = ReportEvery1000 - 1;
    if (ReportEvery1000 == 0)
    {
      int graphNum = TrainingCycle / 100;
      int graphE1 = Error * 1000;
      int graphE = map(graphE1, 3, 80, 47, 0);

      Serial.println();
      Serial.println();
      Serial.print("TrainingCycle: ");
      Serial.print(TrainingCycle);
      Serial.print("  Error = ");
      Serial.println(Error, 5);
      Serial.print("  Graph Num: ");
      Serial.print(graphNum);
      Serial.print("  Graph Error1 = ");
      Serial.print(graphE1);
      Serial.print("  Graph Error = ");
      Serial.println(graphE);

      toTerminal();

      if (TrainingCycle == 1)
      {
        ReportEvery1000 = 99;
      }
      else
      {
        ReportEvery1000 = 100;
      }
    }

    /******************************************************************
      If error rate is less than pre-determined threshold then end
    ******************************************************************/

    if (Error < Success)
      break;
  }
}

void drive_nn()
{

  int num;
  int farDist = 30;
  int closeDist = 10;
  float TestInput[] = {0, 0, 0};

  int Left = distanceArray[SONAR_LEFT];     // Collect sonar distances.
  int Middle = distanceArray[SONAR_MIDDLE]; // Collect sonar distances.
  int Right = distanceArray[SONAR_RIGHT];   // Collect sonar distances.

  Serial.print("Array: ");
  Serial.print(distanceArray[0]);
  Serial.print(" ");
  Serial.print(distanceArray[1]);
  Serial.print(" ");
  Serial.print(distanceArray[2]);
  Serial.println();

  //CONVERT TO FLOAT BETWEEN 0 AND 1
  //0 = clear
  //1 = collision DECTECTED

  if (isClear(Left))
  {
    Left = 0;
  }
  else
  {
    Left = 1;
  }
  if (isClear(Middle))
  {
    Middle = 0;
  }
  else
  {
    Middle = 1;
  }
  if (isClear(Right))
  {
    Right = 0;
  }
  else
  {
    Right = 1;
  }

  // Left = map(Left, 0, 100, 0, 1);
  // Middle = map(Middle, 0, 100, 0, 1);
  // Right = map(Right, 0, 100, 0, 1);

  // Left = constrain(Left, 0, 100);
  // Middle = constrain(Middle, 0, 100);
  // Right = constrain(Right, 0, 100);

  // TestInput[0] = float(Left) / 100;
  // TestInput[1] = float(Middle) / 100;
  // TestInput[2] = float(Right) / 100;

  TestInput[0] = float(Left);
  TestInput[1] = float(Middle);
  TestInput[2] = float(Right);

  Serial.print("Input: ");
  Serial.print(TestInput[2], 2);
  Serial.print("\t");
  Serial.print(TestInput[1], 2);
  Serial.print("\t");
  Serial.println(TestInput[0], 2);

  InputToOutput(TestInput[0], TestInput[1], TestInput[2]); //INPUT to ANN to obtain OUTPUT

  float A = Output[0];
  float B = Output[1];

  // A = int(A);
  // B = int(B);

  Serial.print("Output A = ");
  Serial.println(A);
  Serial.print("Output B = ");
  Serial.println(B);

  A = round(A);
  B = round(B);

  Serial.print("Rounded Output A = ");
  Serial.println(A);
  Serial.print("Rounded Output B = ");
  Serial.println(B);

  A = int(A);
  B = int(B);
  Serial.print("Int Output A = ");
  Serial.println(A);
  Serial.print("Int Output B = ");
  Serial.println(B);

  if (A && B == 1)
  {
    directionControl.directionSelect(directionControl.direction::forward);
  }
  if (A == 1 && B == 0)
  {
    directionControl.directionSelect(directionControl.direction::left);
  }
  if (A == 0 && B == 1)
  {
    directionControl.directionSelect(directionControl.direction::right);
  }
  if (A && B == 0)
  {
    directionControl.directionSelect(directionControl.direction::reverse);
  }

  delay(50);
}

void toTerminal()
{

  for (p = 0; p < PatternCount; p++)
  {
    Serial.println();
    Serial.print("  Training Pattern: ");
    Serial.println(p);
    Serial.print("  Input ");
    for (i = 0; i < InputNodes; i++)
    {
      Serial.print(Input[p][i], DEC);
      Serial.print(" ");
    }
    Serial.print("  Target ");
    for (i = 0; i < OutputNodes; i++)
    {
      Serial.print(Target[p][i], DEC);
      Serial.print(" ");
    }
    /******************************************************************
* Compute hidden layer activations
******************************************************************/

    for (i = 0; i < HiddenNodes; i++)
    {
      Accum = HiddenWeights[InputNodes][i];
      for (j = 0; j < InputNodes; j++)
      {
        Accum += Input[p][j] * HiddenWeights[j][i];
      }
      Hidden[i] = 1.0 / (1.0 + exp(-Accum));
    }

    /******************************************************************
* Compute output layer activations and calculate errors
******************************************************************/

    for (i = 0; i < OutputNodes; i++)
    {
      Accum = OutputWeights[HiddenNodes][i];
      for (j = 0; j < HiddenNodes; j++)
      {
        Accum += Hidden[j] * OutputWeights[j][i];
      }
      Output[i] = 1.0 / (1.0 + exp(-Accum));
    }
    Serial.print("  Output ");
    for (i = 0; i < OutputNodes; i++)
    {
      Serial.print(Output[i], 5);
      Serial.println(" ");
    }
  }
}

void InputToOutput(float In1, float In2, float In3)
{
  float TestInput[] = {0, 0, 0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;

  /******************************************************************
    Compute hidden layer activations
  ******************************************************************/

  for (i = 0; i < HiddenNodes; i++)
  {
    Accum = HiddenWeights[InputNodes][i];
    for (j = 0; j < InputNodes; j++)
    {
      Accum += TestInput[j] * HiddenWeights[j][i];
    }
    Hidden[i] = 1.0 / (1.0 + exp(-Accum));
  }

  /******************************************************************
    Compute output layer activations and calculate errors
  ******************************************************************/

  for (i = 0; i < OutputNodes; i++)
  {
    Accum = OutputWeights[HiddenNodes][i];
    for (j = 0; j < HiddenNodes; j++)
    {
      Accum += Hidden[j] * OutputWeights[j][i];
    }
    Output[i] = 1.0 / (1.0 + exp(-Accum));
  }

  Serial.print("  Output ");
  for (i = 0; i < OutputNodes; i++)
  {
    Serial.print(Output[i], 5);
    Serial.print(" ");
  }
  Serial.println();
}
