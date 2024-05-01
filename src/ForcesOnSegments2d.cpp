#include <iostream>
#include <utility>
#include <tuple>
#include <cmath>
#include "Rods.hpp"
#include "ParametersForRods.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"

ForcesOnSegments2d::ForcesOnSegments2d(ParametersForRods parameter, double interaction_strength) : parameter(parameter), interaction_strength(interaction_strength)
{
    double num_of_rods = parameter.num_of_rods;
    double num_of_segments = parameter.num_of_segments;

    rod_force_y.resize(num_of_rods);
    rod_force_x.resize(num_of_rods);
    rod_torque.resize(num_of_rods);
    segment_force_x.resize(num_of_rods);
    segment_force_y.resize(num_of_rods);
    for (auto &&i : segment_force_x)
    {
        i.resize(num_of_segments);
    }
    for (auto &&i : segment_force_y)
    {
        i.resize(num_of_segments);
    }

    inversed_diameter_of_segment = 1. / parameter.diameter_of_segment;
}

ForcesOnSegments2d::~ForcesOnSegments2d()
{
}

std::pair<
    std::vector<std::vector<double>>,
    std::vector<std::vector<double>>
> ForcesOnSegments2d::getForcesOnSegments2d() const
{
    return {segment_force_x, segment_force_y};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> ForcesOnSegments2d::getForcesOnRods2d() const
{
    return {rod_force_x, rod_force_y, rod_torque};
}

void ForcesOnSegments2d::calcForcesOnRods2d(const Rods& rods)
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

void ForcesOnSegments2d::calcRodRodYukawaForces(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < segment_force_x.size(); rod_i++) {
        for (size_t rod_k = rod_i + 1; rod_k < segment_force_x.size(); rod_k++) {
            for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
                for (size_t segment_l = 0; segment_l < segment_force_x.at(rod_k).size(); segment_l++) {
                    auto [x1, y1] = rods.getPositionOfSegment(rod_i, segment_j);
                    auto [x2, y2] = rods.getPositionOfSegment(rod_k, segment_l);
                    double dx = x1 - x2;
                    double dy = y1 - y2;
                    double squared_distance = CalcSquaredDistance(dx, dy);
                    double distance = std::sqrt(squared_distance);

                    // cut-off
                    if (distance > 2.0) continue;

                    double force_exp =
                        interaction_strength
                        * std::exp(- distance * inversed_diameter_of_segment)
                        * (inversed_diameter_of_segment + 1.0 / distance)
                        / squared_distance;

                    segment_force_x.at(rod_i).at(segment_j) += dx * force_exp;
                    segment_force_y.at(rod_i).at(segment_j) += dy * force_exp;
                    segment_force_x.at(rod_k).at(segment_l) -= dx * force_exp;
                    segment_force_y.at(rod_k).at(segment_l) -= dy * force_exp;
                }
            }
        }
    }
}

void ForcesOnSegments2d::calcRodRodYukawaForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y)
{
    for (size_t rod_i = 0; rod_i < segment_force_x.size(); rod_i++) {
        for (size_t rod_k = rod_i + 1; rod_k < segment_force_x.size(); rod_k++) {
            for (size_t segment_j = 0; segment_j < segment_force_x.at(rod_i).size(); segment_j++) {
                for (size_t segment_l = 0; segment_l < segment_force_x.at(rod_k).size(); segment_l++) {
                    auto [x1, y1] = rods.getPositionOfSegment(rod_i, segment_j);
                    auto [x2, y2] = rods.getPositionOfSegment(rod_k, segment_l);
                    double dx = x1 - nearestInPeriodic(x1, x2, min_x, max_x);
                    double dy = y1 - nearestInPeriodic(y1, y2, min_y, max_y);
                    double squared_distance = CalcSquaredDistance(dx, dy);
                    double distance = std::sqrt(squared_distance);

                    // cut-off
                    if (distance > 2.0) continue;

                    double force_exp =
                        interaction_strength
                        * std::exp(- distance * inversed_diameter_of_segment)
                        * (inversed_diameter_of_segment + 1.0 / distance)
                        / squared_distance;

                    segment_force_x.at(rod_i).at(segment_j) += dx * force_exp;
                    segment_force_y.at(rod_i).at(segment_j) += dy * force_exp;
                    segment_force_x.at(rod_k).at(segment_l) -= dx * force_exp;
                    segment_force_y.at(rod_k).at(segment_l) -= dy * force_exp;
                }
            }
        }
    }
}

double ForcesOnSegments2d::nearestInPeriodic(double x1, double x2, double min_x, double max_x)   // 周期的境界により生じる像の内最短距離にあるものを探索
{
    double dx = x1 - x2;
    double period = max_x - min_x;

    // original is nearest
    if (std::abs(2 * dx) < period) return x2;

    // which mirror image is nearest
    if (dx < 0) {
        period = - period;
    }
    // mirror is nearest
    return x2 + period;
}

void ForcesOnSegments2d::calcRodRodVicsekForces(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        double sum_of_orientation_x = 0.;
        double sum_of_orientation_y = 0.;
        for (size_t rod_j = 0; rod_j < rod_torque.size(); rod_j++) {
            auto [x1, y1] = rods.getPositionOfCenterOfMass(rod_i);
            auto [x2, y2] = rods.getPositionOfCenterOfMass(rod_j);
            double dx = x1 - x2;
            double dy = y1 - y2;
            double squared_distance = CalcSquaredDistance(dx, dy);
            double distance = std::sqrt(squared_distance);

            // cut-off
            if (distance > 5.0) continue;

            auto [cos, sin] = rods.getOrientation(rod_j);
            sum_of_orientation_x += cos;
            sum_of_orientation_y += sin;
        }
        double averaged_angle = std::atan2(sum_of_orientation_y, sum_of_orientation_x);
        double delta_angle = averaged_angle - rods.getAngle(rod_i);
        if (delta_angle > 0) {
            delta_angle = std::fmod(delta_angle + M_PI, 2 * M_PI) - M_PI;
        } else {
            delta_angle = std::fmod(delta_angle - M_PI, 2 * M_PI) + M_PI;
        }
        rod_torque.at(rod_i) += interaction_strength * delta_angle;
    }
}

void ForcesOnSegments2d::calcRodRodVicsekForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        double sum_of_orientation_x = 0.;
        double sum_of_orientation_y = 0.;
        for (size_t rod_j = 0; rod_j < rod_torque.size(); rod_j++) {
            auto [x1, y1] = rods.getPositionOfCenterOfMass(rod_i);
            auto [x2, y2] = rods.getPositionOfCenterOfMass(rod_j);
            double dx = x1 - nearestInPeriodic(x1, x2, min_x, max_x);
            double dy = y1 - nearestInPeriodic(y1, y2, min_y, max_y);
            double squared_distance = CalcSquaredDistance(dx, dy);
            double distance = std::sqrt(squared_distance);

            // cut-off
            if (distance > 5.0) continue;

            auto [cos, sin] = rods.getOrientation(rod_j);
            sum_of_orientation_x += cos;
            sum_of_orientation_y += sin;
        }
        double averaged_angle = std::atan2(sum_of_orientation_y, sum_of_orientation_x);
        double delta_angle = averaged_angle - rods.getAngle(rod_i);
        if (delta_angle > 0) {
            delta_angle = std::fmod(delta_angle + M_PI, 2 * M_PI) - M_PI;
        } else {
            delta_angle = std::fmod(delta_angle - M_PI, 2 * M_PI) + M_PI;
        }
        rod_torque.at(rod_i) += interaction_strength * delta_angle;
    }
}

void ForcesOnSegments2d::calcRodRodPolarSinusoidalForces(const Rods& rods)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        for (size_t rod_j = rod_i + 1; rod_j < rod_torque.size(); rod_j++) {
            auto [x1, y1] = rods.getPositionOfCenterOfMass(rod_i);
            auto [x2, y2] = rods.getPositionOfCenterOfMass(rod_j);
            double dx = x1 - x2;
            double dy = y1 - y2;
            double squared_distance = CalcSquaredDistance(dx, dy);
            double distance = std::sqrt(squared_distance);

            // cut-off
            if (distance > 5.0) continue;

            double polar_torque = - std::sin(rods.getAngle(rod_i) - rods.getAngle(rod_j));
            rod_torque.at(rod_i) += interaction_strength * polar_torque;
            rod_torque.at(rod_j) -= interaction_strength * polar_torque;
        }
    }
}

void ForcesOnSegments2d::calcRodRodPolarSinusoidalForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        for (size_t rod_j = 0; rod_j < rod_torque.size(); rod_j++) {
            auto [x1, y1] = rods.getPositionOfCenterOfMass(rod_i);
            auto [x2, y2] = rods.getPositionOfCenterOfMass(rod_j);
            double dx = x1 - nearestInPeriodic(x1, x2, min_x, max_x);
            double dy = y1 - nearestInPeriodic(y1, y2, min_y, max_y);
            double squared_distance = CalcSquaredDistance(dx, dy);
            double distance = std::sqrt(squared_distance);

            // cut-off
            if (distance > 5.0) continue;

            double polar_torque = - std::sin(rods.getAngle(rod_i) - rods.getAngle(rod_j));
            rod_torque.at(rod_i) += interaction_strength * polar_torque;
            rod_torque.at(rod_j) -= interaction_strength * polar_torque;
        }
    }
}

void ForcesOnSegments2d::calcRLAgentArtificialForces(const Rods& rods, std::vector<double> actions)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        //double polar_torque = - std::sin(rods.getAngle(rod_i) - actions.at(rod_i));
        rod_torque.at(rod_i) += interaction_strength * actions.at(rod_i);
    }
}

void ForcesOnSegments2d::calcRLAgentArtificialForcesWithPeriodic(const Rods& rods, std::vector<double> actions, double min_x, double max_x, double min_y, double max_y)
{
    for (size_t rod_i = 0; rod_i < rod_torque.size(); rod_i++) {
        //double polar_torque = - std::sin(rods.getAngle(rod_i) - actions.at(rod_i));
        rod_torque.at(rod_i) += interaction_strength * actions.at(rod_i);
    }
}

void ForcesOnSegments2d::ClearCalculatedForces()
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
