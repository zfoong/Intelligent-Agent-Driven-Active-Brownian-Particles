#include <iostream>
#include <vector>
#include <cmath>

#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "ParametersForForces.hpp"
#include "RodsCircularObstacleYukawaForces.hpp"
#include "util.hpp"

RodsCircularObstacleYukawaForces::RodsCircularObstacleYukawaForces(
    ParametersForRods parameter,
    ParametersForCircularBoundary &obstacle_parameter,
    ParametersForForces &forces_parameter
) : rod_force_x(parameter.num_of_rods, 0.),
    rod_force_y(parameter.num_of_rods, 0.),
    rod_torque(parameter.num_of_rods, 0.),
    segment_force_x(parameter.num_of_rods, std::vector<double>(parameter.num_of_segments, 0.)),
    segment_force_y(parameter.num_of_rods, std::vector<double>(parameter.num_of_segments, 0.)),

    radius_of_circular_obstacle(obstacle_parameter.getRadius()),
    center_of_circle_x(obstacle_parameter.getPositionOfCenter().first),
    center_of_circle_y(obstacle_parameter.getPositionOfCenter().second),

    cut_off_distance(forces_parameter.cut_off_distance),
    interaction_strength(forces_parameter.interaction_strength),
    inv_diameter_of_segment(1. / parameter.diameter_of_segment)
{
}

void RodsCircularObstacleYukawaForces::setInteractionStrength(double interaction_strength)
{
    this->interaction_strength = interaction_strength;
}

void RodsCircularObstacleYukawaForces::setRadiusOfCircularObstacle(double radius_of_circular_obstacle)
{
    this->radius_of_circular_obstacle = radius_of_circular_obstacle;
}

void RodsCircularObstacleYukawaForces::setCenterOfCircularObstacle(double center_of_circle_x, double center_of_circle_y)
{
    this->center_of_circle_x = center_of_circle_x;
    this->center_of_circle_y = center_of_circle_y;
}

double RodsCircularObstacleYukawaForces::getInteractionStrength() const
{
    return this->interaction_strength;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> RodsCircularObstacleYukawaForces::getForcesOnRods() const
{
    return {rod_force_x, rod_force_y, rod_torque};
}

void RodsCircularObstacleYukawaForces::compute(const Rods& rods)
{
    computeForcesOnSegments(rods);
    computeForcesOnRods(rods);
}
void RodsCircularObstacleYukawaForces::computeForcesOnSegments(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < segment_force_x.size(); rod_i++) {
        for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
            // calculate the distance between circular boundary and the center of segment j in rod i
            auto [segment_x, segment_y] = rods.getPositionOfSegment(rod_i, segment_j);
            const double dx_from_center_of_circle_x = segment_x - center_of_circle_x;
            const double dy_from_center_of_circle_y = segment_y - center_of_circle_y;
            double distance_between_segment_and_center_of_circle = std::sqrt(CalcSquaredDistance(dx_from_center_of_circle_x, dy_from_center_of_circle_y));
            double distance = distance_between_segment_and_center_of_circle - radius_of_circular_obstacle;

            // cut-off
            if (distance > cut_off_distance) continue;

            const double force =
                interaction_strength
                * std::exp(- 2.0 * distance * inv_diameter_of_segment)
                * (inv_diameter_of_segment + 0.5 / distance)
                / distance_between_segment_and_center_of_circle
                / distance;

            segment_force_x.at(rod_i).at(segment_j) = dx_from_center_of_circle_x * force;
            segment_force_y.at(rod_i).at(segment_j) = dy_from_center_of_circle_y * force;
        }
    }
}

void RodsCircularObstacleYukawaForces::computeForcesOnRods(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < rod_force_x.size(); rod_i++) {
        auto [xg, yg] = rods.getPositionOfCenterOfMass(rod_i);
        for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
            auto [x, y] = rods.getPositionOfSegment(rod_i, segment_j);
            rod_force_x.at(rod_i) += segment_force_x.at(rod_i).at(segment_j);
            rod_force_y.at(rod_i) += segment_force_y.at(rod_i).at(segment_j);
            rod_torque .at(rod_i) += (x - xg) * segment_force_y.at(rod_i).at(segment_j)
                                   - (y - yg) * segment_force_x.at(rod_i).at(segment_j);
        }
    }
}

void RodsCircularObstacleYukawaForces::clear()
{
    for (size_t rod_i = 0; rod_i < rod_force_x.size(); rod_i++) {
        rod_force_x.at(rod_i) = 0.;
        rod_force_y.at(rod_i) = 0.;
        rod_torque .at(rod_i) = 0.;
        for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
            segment_force_x.at(rod_i).at(segment_j) = 0.;
            segment_force_y.at(rod_i).at(segment_j) = 0.;
        }
    }
}
