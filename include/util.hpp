#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <torch/torch.h>
#include <vector>

using json = nlohmann::json;

double CalcSquaredDistance(double dx, double dy);
void DisplayProgress(int steps, int total_steps);
void FileOpenErrorCheck(FILE *fp, std::string filename);
void WriteParametersJson(std::string dirname, std::string filename, json parameter_json);
torch::Tensor vector2DToTensor(const std::vector<std::vector<double>>& vec);
