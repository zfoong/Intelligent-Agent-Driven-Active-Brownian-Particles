#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <nlohmann/json.hpp>
#include "util.hpp"
#include <torch/torch.h>
#include <vector>
#include <deque>
#include <numeric>
#include <cmath>

using json = nlohmann::json;

double CalcSquaredDistance(double dx, double dy)
{
    return dx * dx + dy * dy;
}

void DisplayProgress(int steps, int total_steps)
{
    std::string progress_status;
    progress_status.reserve(100);
    progress_status.append("Progress is ")
        .append(std::to_string(steps / (total_steps / 100))).append("%: ")
        .append(std::to_string(steps)).append(" steps in ")
        .append(std::to_string(total_steps)).append(" total steps");
    std::cout << progress_status << std::endl;
}

void FileOpenErrorCheck(FILE *fp, std::string filename)
{
    if (fp == NULL) {   // エラーチェック
        std::cerr << "error: cannot open " + filename << std::endl;
        exit(-1);
    }
}

void WriteParametersJson(std::string dirname, std::string filename, json parameter_json)
{
    // write prettified JSON to another file
    std::string fullpath_filename = dirname + filename;
    std::ofstream parameter_output(fullpath_filename);
    parameter_output << std::setw(2) << parameter_json << std::endl;
}

torch::Tensor vector2DToTensor(const std::vector<std::vector<double>>& vec) {
    // Calculate the total size of the 2D vector
    int total_size = 0;
    for (const auto& subVec : vec) {
        total_size += subVec.size();
    }

    // Flatten the 2D vector into a 1D vector
    std::vector<double> flattened(total_size);
    int index = 0;
    for (const auto& subVec : vec) {
        std::copy(subVec.begin(), subVec.end(), flattened.begin() + index);
        index += subVec.size();
    }

    // Convert the 1D vector to a torch::Tensor
    torch::Tensor tensor = torch::from_blob(flattened.data(), {static_cast<long>(vec.size()), static_cast<long>(vec[0].size())}, torch::kDouble);

    return tensor.clone(); // Return a clone of the tensor to make it own its memory
}

std::vector<double> calculateWeightedAverageReward(const std::vector<double>& new_reward, std::deque<std::vector<double>> reward_history, const int reward_history_length)
{
    if (reward_history.size() >= reward_history_length) {
        reward_history.pop_front();
    }
    reward_history.push_back(new_reward);

    std::vector<double> weighted_reward_sum(new_reward.size(), 0.0);
    int i = 0;
    for (auto it = reward_history.rbegin(); it != reward_history.rend(); ++it, ++i) {
        double weight = std::exp(-0.05 * i); // Exponential decay factor
        for (size_t j = 0; j < it->size(); j++) {
            weighted_reward_sum[j] += (*it)[j] * weight;
        }
    }

    double total_weights = (1.0 - std::exp(-0.05 * reward_history.size())) / (1.0 - std::exp(-0.05)); // Normalization factor for weights
    for (size_t j = 0; j < weighted_reward_sum.size(); j++) {
        weighted_reward_sum[j] /= total_weights;
    }

    return weighted_reward_sum;
}
