#include <iostream>
#include "ParametersForForces.hpp"

ParametersForForces::ParametersForForces(
    double interaction_strength,
    double cut_off_distance
) : interaction_strength(interaction_strength),
    cut_off_distance(cut_off_distance),
    squared_cut_off_distance((cut_off_distance == -1.0) ? -0.1 : cut_off_distance * cut_off_distance)
{
}

void ParametersForForces::displayParameters() const
{
    std::cout << "interaction strength = " << this->interaction_strength << std::endl;
    std::cout << "cut off distance = " << this->cut_off_distance << std::endl;
    std::cout << "squared cut off distance = " << this->squared_cut_off_distance << std::endl;
}
