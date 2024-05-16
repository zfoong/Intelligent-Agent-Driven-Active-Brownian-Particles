#pragma once

class ParametersForForces
{
public:
    double interaction_strength;
    double cut_off_distance;
    double squared_cut_off_distance;

    ParametersForForces(double interaction_strength, double cut_off_distance = -1.0);
    ~ParametersForForces() = default;

    void displayParameters() const;
};
