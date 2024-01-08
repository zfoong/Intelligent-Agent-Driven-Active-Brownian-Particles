#pragma once
#include <tuple>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class ParametersForRods
{
public:
    double length;
    double diameter_of_segment;
    double aspect_ratio;
    int num_of_rods;
    int num_of_segments;
    double viscosity_parallel;
    double viscosity_vertical;
    double viscosity_rotation;
    double fluidity_parallel;
    double fluidity_vertical;
    double fluidity_rotation;

    ParametersForRods();
    ParametersForRods(
        double length,
        double diameter_of_segment,
        double viscosity,
        int num_of_rods,
        int num_of_segments
    );
    ParametersForRods(
        double length,
        double diameter_of_segment,
        double viscosity,
        int num_of_rods
    );
    ParametersForRods(json parameter_json);
    ~ParametersForRods();

    static int CalcNumberOfSegment(double aspect_ratio);
    std::tuple<double, double, double> CalcViscosities(double viscosity, double aspect_ratio) const;
    void SetFluidities(double fluidity_parallel, double fluidity_vertical, double fluidity_rotation);
    void ConvertTimeScale(double scalar);
    void DisplayParameters() const;
};

class ParametersForActiveRods : public ParametersForRods
{
public:
    double active_force;
    double self_propelling_velocity;

    ParametersForActiveRods(ParametersForRods parameter, double active_force);
    ParametersForActiveRods(json parameter_json);
    void ConvertTimeScale(double scalar);
    void DisplayParameters() const;
};

class ParametersForIntelligentActiveRods : public ParametersForRods
{
public:
    double active_force;
    double self_propelling_velocity;

    ParametersForIntelligentActiveRods(ParametersForRods parameter, double active_force);
    ParametersForIntelligentActiveRods(json parameter_json);
    void ConvertTimeScale(double scalar);
    void DisplayParameters() const;
};