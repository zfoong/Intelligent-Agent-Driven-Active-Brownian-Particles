#pragma once
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "CircularBoundary.hpp"
#include "Periodic.hpp"

class ObstaclesInPeriodicCompression : PeriodicCompression
{
protected:
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles;
    std::vector<CircularBoundary> obstacles;

public:
    ObstaclesInPeriodicCompression(Rods &rods, ParametersForRods &parameter_for_rods, std::vector<ParametersForCircularBoundary> &parameters_for_obstacles);
    ~ObstaclesInPeriodicCompression();
    void Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_linear_dimension, double final_linear_dimension);
};

class ObstaclesSteepeningInPeriodic
{
protected:
    Rods &rods;
    ParametersForRods &parameter;
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles;
    std::vector<CircularBoundary> obstacles;

public:
    ObstaclesSteepeningInPeriodic(Rods &rods, ParametersForRods &parameter_for_rods, std::vector<ParametersForCircularBoundary> &parameters_for_obstacles);
    ~ObstaclesSteepeningInPeriodic();
    void Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_interaction_strength, double final_interaction_strength);
};
