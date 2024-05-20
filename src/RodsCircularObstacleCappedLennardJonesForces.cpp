#include <iostream>
#include <vector>
#include <cmath>

#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "ParametersForForces.hpp"
#include "RodsCircularObstacleCappedLennardJonesForces.hpp"
#include "util.hpp"

RodsCircularObstacleCappedLennardJonesForces::RodsCircularObstacleCappedLennardJonesForces(
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

    cut_off_distance(parameter.diameter_of_segment / 2 + this->radius_of_circular_obstacle),    // r_min
    cut_off_distance_2(cut_off_distance * cut_off_distance),
    sigma(std::pow(2., 1. / 3.) * this->cut_off_distance),
    sigma_2(sigma * sigma),
    alpha_2(std::pow(2., 1. / 3.) * sigma_2 - cut_off_distance_2),    // alpha = (2^(1/3)sigma^2 - r_min^2)^(1/2)
    interaction_strength(
        24. * forces_parameter.interaction_strength  // 24 is originated from the expression of Lennard-Jones potential
    )
{
}

void RodsCircularObstacleCappedLennardJonesForces::setInteractionStrength(double interaction_strength)
{
    this->interaction_strength = interaction_strength;
}

void RodsCircularObstacleCappedLennardJonesForces::setRadiusOfCircularObstacle(double radius_of_circular_obstacle)
{
    this->radius_of_circular_obstacle = radius_of_circular_obstacle;
}

void RodsCircularObstacleCappedLennardJonesForces::setCenterOfCircularObstacle(double center_of_circle_x, double center_of_circle_y)
{
    this->center_of_circle_x = center_of_circle_x;
    this->center_of_circle_y = center_of_circle_y;
}

double RodsCircularObstacleCappedLennardJonesForces::getInteractionStrength() const
{
    return this->interaction_strength;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> RodsCircularObstacleCappedLennardJonesForces::getForcesOnRods() const
{
    return {rod_force_x, rod_force_y, rod_torque};
}

void RodsCircularObstacleCappedLennardJonesForces::compute(const Rods& rods)
{
    computeForcesOnSegments(rods);
    computeForcesOnRods(rods);
}
void RodsCircularObstacleCappedLennardJonesForces::computeForcesOnSegments(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < segment_force_x.size(); rod_i++) {
        for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
            // calculate the distance between circular boundary and the center of segment j in rod i
            auto [segment_x, segment_y] = rods.getPositionOfSegment(rod_i, segment_j);
            const double dx_from_center_of_circle_x = segment_x - center_of_circle_x;
            const double dy_from_center_of_circle_y = segment_y - center_of_circle_y;
            // squared distance between the center of segment and center of circle
            const double squared_distance = CalcSquaredDistance(dx_from_center_of_circle_x, dy_from_center_of_circle_y);

            // cut-off
            if (squared_distance > cut_off_distance_2) continue;

            // 1 / (alpha^2 + r^2)
            const double inv_alpha_2_and_distance_2 = 1. / (alpha_2 + squared_distance);
            // sigma^2 / (alpha^2 + r^2)
            const double sigma_2_inv_alpha_2_and_distance_2 = sigma_2 / (alpha_2 + squared_distance);
            // (sigma^2 / (alpha^2 + r^2))^3
            const double sigma_2_inv_alpha_2_and_distance_2_3 = sigma_2_inv_alpha_2_and_distance_2 * sigma_2_inv_alpha_2_and_distance_2 * sigma_2_inv_alpha_2_and_distance_2;

            const double force =
                interaction_strength
                * inv_alpha_2_and_distance_2
                * sigma_2_inv_alpha_2_and_distance_2_3
                * (2. * sigma_2_inv_alpha_2_and_distance_2_3 - 1.0);

            segment_force_x.at(rod_i).at(segment_j) = dx_from_center_of_circle_x * force;
            segment_force_y.at(rod_i).at(segment_j) = dy_from_center_of_circle_y * force;
        }
    }
}

void RodsCircularObstacleCappedLennardJonesForces::computeForcesOnRods(const Rods& rods)
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

void RodsCircularObstacleCappedLennardJonesForces::clear()
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
