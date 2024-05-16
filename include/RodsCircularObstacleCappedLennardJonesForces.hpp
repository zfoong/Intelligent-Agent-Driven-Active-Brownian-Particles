#pragma once
#include <vector>
#include <tuple>

#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "ParametersForForces.hpp"

class RodsCircularObstacleCappedLennardJonesForces
{
protected:
    // forces acting on rod i
    std::vector<double> rod_force_x, rod_force_y, rod_torque;
    // forces acting on rod i's segment j
    std::vector<std::vector<double>> segment_force_x, segment_force_y;

    double radius_of_circular_obstacle;
    double center_of_circle_x, center_of_circle_y;

    // cut_off_distance is the distance at which the segment-obstacle potential energy V is exactly zero, cut_off_distance_2 = cut_off_distance^2
    double cut_off_distance, cut_off_distance_2;
    // sigma and alpha characterize the capping of the Lennard-Jones potential
    // and is determined from one to the other, sigma_2 = sigma^2 and alpha_2 = alpha^2
    double sigma, sigma_2; double alpha_2;
    // coefficient of the Lennard-Jones potential usually denoted as 24ε
    double interaction_strength;

public:
    RodsCircularObstacleCappedLennardJonesForces(
        ParametersForRods parameter,
        ParametersForCircularBoundary &obstacle_parameter,
        ParametersForForces &forces_parameter
    );
    ~RodsCircularObstacleCappedLennardJonesForces() = default;

    void setInteractionStrength(double interaction_strength);
    void setRadiusOfCircularObstacle(double radius_of_circular_obstacle);
    double getInteractionStrength() const;
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getForcesOnRods() const;

    void compute(const Rods &rods);
    void computeForcesOnSegments(const Rods &rods);
    void computeForcesOnRods(const Rods &rods);
    void clear();
};
