#pragma once
#include <nlohmann/json.hpp>

#include "Rods.hpp"
#include "ParametersForRods.hpp"

using json = nlohmann::json;

class Periodic
{
protected:
    ActiveRods &rods;
    ParametersForActiveRods &parameter;

public:

    Periodic(ActiveRods &active_rods, ParametersForActiveRods &parameter_for_active_rods);
    ~Periodic();
    void Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale);
};

class PeriodicCompression
{
protected:
    Rods &rods;
    ParametersForRods &parameter;

public:

    PeriodicCompression(Rods &rods, ParametersForRods &parameter_for_rods);
    ~PeriodicCompression();
    void Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_linear_dimension, double final_linear_dimension);
};
