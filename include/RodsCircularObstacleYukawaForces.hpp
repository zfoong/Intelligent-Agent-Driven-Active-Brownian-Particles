#pragma once
#include <vector>
#include <tuple>

#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "ParametersForForces.hpp"

class RodsCircularObstacleYukawaForces
{
protected:
    // forces acting on rod i
    std::vector<double> rod_force_x, rod_force_y, rod_torque;
    // forces acting on rod i's segment j
    std::vector<std::vector<double>> segment_force_x, segment_force_y;

    double radius_of_circular_obstacle;
    double center_of_circle_x, center_of_circle_y;

    double cut_off_distance;
    double interaction_strength;
    double inv_diameter_of_segment;

public:
    RodsCircularObstacleYukawaForces(
        ParametersForRods parameter,
        ParametersForCircularBoundary &obstacle_parameter,
        ParametersForForces &forces_parameter
    );
    ~RodsCircularObstacleYukawaForces() = default;

    void setInteractionStrength(double interaction_strength);
    void setRadiusOfCircularObstacle(double radius_of_circular_obstacle);
    void setCenterOfCircularObstacle(double center_of_circle_x, double center_of_circle_y);
    double getInteractionStrength() const;
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getForcesOnRods() const;

    void compute(const Rods &rods);
    void computeForcesOnSegments(const Rods &rods);
    void computeForcesOnRods(const Rods &rods);
    void clear();
};
