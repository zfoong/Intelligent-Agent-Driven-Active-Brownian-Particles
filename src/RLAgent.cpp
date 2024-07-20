#include <vector>
#include <iostream>
#include <cmath>
#include <torch/torch.h>
#include <string>
#include <random>
#include <algorithm> 
#include <fstream>
#include <sstream>
#include <functional>
#include <numeric>
#include "util.hpp"
#include "RLAgent.hpp"

int argmax(double*, int);
double arrmax(double*, int);

RLAgent::RLAgent(double lr, double ep, double epDecay, double epMin, double df)
    : learningRate(lr), maxlr(lr), epsilon(ep), epsilonDecay(epDecay), minEpsilon(epMin), discountFactor(df), minlr(0), learningRateDecay(0.2), maxEpsilon(ep), radiansPiece(RADIANS / (double)K) {
    std::fill(std::begin(SVTable), std::end(SVTable), 0);
    std::fill(std::begin(sortedSVTable), std::end(sortedSVTable), 0);
    std::fill(&TPMatrix[0][0][0], &TPMatrix[0][0][0] + sizeof(TPMatrix) / sizeof(TPMatrix[0][0][0]), 0);
    std::fill(&DTable[0][0][0], &DTable[0][0][0] + sizeof(DTable) / sizeof(DTable[0][0][0]), 0);
}

// sort optimum state
void RLAgent::SortStateValueList() {
    std::vector<int> V(K);
    std::iota(V.begin(), V.end(), 0);
    sort(V.begin(), V.end(), [&](int i, int j) {return SVTable[i]>SVTable[j]; });
    for (int k = 0; k < V.size(); k++)
        sortedSVTable[k] = V[k];
}

// state value function update via TD-Learning
void RLAgent::UpdateSVTable(double state, int actionID, double reward, double newState) {
    int stateID = StateToIndex(state);
    int newStateID = StateToIndex(newState);
    SVTable[stateID] += learningRate * (reward + discountFactor * SVTable[newStateID] - SVTable[stateID]); // TD-Learning
    DTable[stateID][newStateID][actionID]++; // record state-action occurance in a distribution table
}

// update state value for MAS
void RLAgent::UpdateSVTable(std::vector<double> stateList, std::vector<int> actionIDList, std::vector<double> rewardList, std::vector<double> newStateList) {
    for (int i = 0; i < stateList.size(); i++) {
        UpdateSVTable(stateList[i], actionIDList[i], rewardList[i], newStateList[i]);
    }
}

// model update
void RLAgent::UpdateTPMatrix() {
    int n_d1 = sizeof(TPMatrix) / sizeof(*TPMatrix);
    int n_d2 = sizeof(TPMatrix[n_d1]) / sizeof(*TPMatrix[n_d1]);
    int n_d3 = sizeof(TPMatrix[n_d1][n_d2]) / sizeof(*TPMatrix[n_d1][n_d2]);
    for (int i = 0; i < n_d3; i++) {
        for (int j = 0; j < n_d1; j++) {
            int actionTotal = 0;
            // compute total occurance of action row
            for (int k = 0; k < n_d2; k++) {
                actionTotal += DTable[j][k][i]; 
            }
            // compute probability of state-action to next state
            for (int k = 0; k < n_d2; k++) {
                TPMatrix[j][k][i] = (double)DTable[j][k][i] / (double)actionTotal;
            }
        }
    }
}

// action selection function
double RLAgent::ReturnAction(double state, int &actionID) {
    int stateID = StateToIndex(state);
    int count = sizeof(SVTable) / sizeof(SVTable[stateID]);
    if (epsilon >= ((double)rand() / (RAND_MAX))) {
        actionID = rand() % count;
    } else {
        /*TP-Matrix (model) action select. one can search for local available actions with highest value,
        or just sort all state value accrodingly and look for any action agents can take (our approach).
        our approach required less computational operation needed, even it is not intuitive.*/
        double thr = 0.1;
        int index = 0;
        int SVTableCount = sizeof(sortedSVTable) / sizeof(sortedSVTable[0]);
        while (index < SVTableCount) {
            int optimumStateID = sortedSVTable[index];
            actionID = argmax(TPMatrix[stateID][optimumStateID], count);
            if (TPMatrix[stateID][optimumStateID][actionID] > thr) {
                return -(IndexToAction(actionID));
            }
            else {
                index++;
            }
        }
    }
    return -(IndexToAction(actionID)); // append negative sign to flip orientation
}

// action selection for MAS
std::vector<double> RLAgent::ReturnAction(std::vector<double> stateList, std::vector<int> &actionIDList) {
    std::vector<double> actionList;
    for (int i = 0; i < stateList.size(); i++) {
        actionList.push_back(ReturnAction(stateList[i], actionIDList[i]));
    }
    return actionList;
}

void RLAgent::UpdateEpsilonDecay(double t, double totalTime) {
    epsilon = minEpsilon + (maxEpsilon - minEpsilon) * exp(-epsilonDecay * t);
}

void RLAgent::UpdateLearningRateDecay(double t, double totalTime) {
    learningRate = minlr + (maxlr - minlr) * exp(-learningRateDecay * t);
}

void RLAgent::setEpsilon(double ep) {
    epsilon = ep;
}

double RLAgent::returnEpsilon() {
    return epsilon;
}

void RLAgent::setLearningRate(double lr) {
    learningRate = lr;
}

double RLAgent::returnLearningRate() {
    return learningRate;
}

// return max value index within array
int argmax(double *arr, int size) {
    return std::distance(arr, std::max_element(arr ,arr + size));
}

// return max value within array
double arrmax(double *arr, int size) {
    double max = arr[0];
    for(int i = 1; i < size; i++)
        if (max < arr[i]) max = arr[i];
    return max;
}

// from state in [-pi, pi) to state index in [0, K]
int RLAgent::StateToIndex(double state) {
    /*
    this help to put external force into the middle of a state piece
    doing so help to avoid the behaviour where flocking agents will going around in a circle
    it is not implemented because previous training was not done using it
    uncomment this to make the simulation look more natural 
    */
    //state += M_PI + (radiansPiece / 2); 
    
    state += M_PI; 
    return floor(state / radiansPiece);
}

// from action in [-pi, pi) to action index in [0, K]
int RLAgent::ActionToIndex(double state) {
    state += M_PI;
    return floor(state / radiansPiece);
}

// from action index in [0, K] to action in [-pi, pi)
double RLAgent::IndexToAction(int index) {
    return (radiansPiece * index) - M_PI;
}

void RLAgent::SaveSVTable(const char* path) {
    std::string fileExt = ".csv";
    std::string filePath = path + fileExt;
    std::ofstream outFile(filePath);
    for (auto& row : SVTable) {
            outFile << row << '\n';
    }
}

void RLAgent::SaveTPMatrix(const char* path) {
    std::string fileExt = ".csv";
    std::string filePath = path + fileExt;
    FILE* pFile = fopen(path, "wb");
    fwrite(TPMatrix, sizeof(TPMatrix), 1, pFile);
    fclose(pFile);

    std::ofstream outFile(filePath);
    int n_d1 = sizeof(TPMatrix) / sizeof(*TPMatrix);
    int n_d2 = sizeof(TPMatrix[n_d1]) / sizeof(*TPMatrix[n_d1]);
    int n_d3 = sizeof(TPMatrix[n_d1][n_d2]) / sizeof(*TPMatrix[n_d1][n_d2]);
    for (int i = 0; i < n_d3; i++) {
        for (int j = 0; j < n_d1; j++) {
            for (int k = 0; k < n_d2; k++) {
                outFile << TPMatrix[j][k][i] << ',';
            }
            outFile << '\n';
        }
        outFile << '\n';
        outFile << '\n';
    }
}

void RLAgent::SaveDTable(const char* path) {
    std::string fileExt = ".csv";
    std::string filePath = path + fileExt;
    FILE* pFile = fopen(path, "wb");
    fwrite(DTable, sizeof(DTable), 1, pFile);
    fclose(pFile);

    std::ofstream outFile(filePath);
    int n_d1 = sizeof(DTable) / sizeof(*DTable);
    int n_d2 = sizeof(DTable[n_d1]) / sizeof(*DTable[n_d1]);
    int n_d3 = sizeof(DTable[n_d1][n_d2]) / sizeof(*DTable[n_d1][n_d2]);
    for (int i = 0; i < n_d3; i++) {
        for (int j = 0; j < n_d1; j++) {
            for (int k = 0; k < n_d2; k++) {
                outFile << DTable[j][k][i] << ',';
            }
            outFile << '\n';
        }
        outFile << '\n';
        outFile << '\n';
    }
}

void RLAgent::LoadSVTable(const char* path) {
    std::string fileExt = ".csv";
    std::string filePath = path + fileExt;
    std::cout << "loading SV Table from svtable.csv" << std::endl;
    std::ifstream file(filePath);
    for (int row = 0; row < K; row++)
    {
        std::string line;
        std::getline(file, line);
        std::stringstream iss(line);    
        iss >> SVTable[row];
    }
}

void RLAgent::LoadTPMatrix(const char* path) {
    FILE* pFile = fopen(path, "rb");
    fread(TPMatrix, sizeof(TPMatrix), 1, pFile);
    fclose(pFile);
}

void RLAgent::LoadDTable(const char* path) {
    FILE* pFile = fopen(path, "rb");
    fread(DTable, sizeof(DTable), 1, pFile);
    fclose(pFile);
}

