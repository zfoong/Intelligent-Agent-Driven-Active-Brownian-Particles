#ifndef RLAGENT_HPP
#define RLAGENT_HPP

#include <vector>
#include <string>
#include <cmath>

static const int K = 11;  // split state space and action space into K pieces of discretized units

class RLAgent {
public:
    RLAgent(double lr = 0.5, double ep = 1, double epDecay = 10, double epMin = 0, double df = 0.9);
    
    const double RADIANS = M_PI * 2;
    double learningRate;
    double minlr;
    double maxlr;
    double learningRateDecay;
    double discountFactor;
    double epsilon;
    double minEpsilon;
    double maxEpsilon;
    double epsilonDecay;
    double radiansPiece;
    double SVTable[K];
    int sortedSVTable[K];
    double TPMatrix[K][K][K];
    int DTable[K][K][K];

    void UpdateSVTable(double state, int actionID, double reward, double newState);
    void UpdateSVTable(std::vector<double> stateList, std::vector<int> actionIDList, std::vector<double> rewardList, std::vector<double> newStateList);
    void SortStateValueList();
    void UpdateTPMatrix();
    double ReturnAction(double state, int &actionID);
    std::vector<double> ReturnAction(std::vector<double> stateList, std::vector<int>& actionIDList);
    void UpdateEpsilonDecay(double t, double totalTime);
    void setEpsilon(double ep);
    double returnEpsilon();
    void UpdateLearningRateDecay(double t, double totalTime);
    void setLearningRate(double lr);
    double returnLearningRate();
    void SaveSVTable(const char* path);
    void SaveTPMatrix(const char* path);
    void SaveDTable(const char* path);
    void LoadSVTable(const char* path);
    void LoadTPMatrix(const char* path);
    void LoadDTable(const char* path);

private:
    int StateToIndex(double state);
    int ActionToIndex(double state);
    double IndexToAction(int index);
};

#endif // RLAGENT_HPP