#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <torch/torch.h>
#include <vector>
#include <deque>
#include <numeric>
#include <cmath>

using json = nlohmann::json;

double CalcSquaredDistance(double dx, double dy);
void DisplayProgress(int steps, int total_steps);
void FileOpenErrorCheck(FILE *fp, std::string filename);
void WriteParametersJson(std::string dirname, std::string filename, json parameter_json);
torch::Tensor vector2DToTensor(const std::vector<std::vector<double>>& vec);
std::vector<double> calculateWeightedAverageReward(const std::vector<double>& new_reward, std::deque<std::vector<double>> reward_history, const int reward_history_length);
