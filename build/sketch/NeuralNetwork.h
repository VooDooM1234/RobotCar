#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#define SONAR_LEFT 0
#define SONAR_MIDDLE 1
#define SONAR_RIGHT 2

void NeuralNetworkSetup();
void train_nn();
void drive_nn(int arr[]);
void toTerminal();

#endif