#pragma once
#include <vector>
#include <utility>
#include <tuple>
#include "Rods.hpp"
#include "ParametersForRods.hpp"

class ForcesOnSegments2d
{
private:
    // forces acting on rod i
    std::vector<double> rod_force_x;
    std::vector<double> rod_force_y;
    std::vector<double> rod_torque;
    // forces acting on rod i's segment j
    std::vector<std::vector<double>> segment_force_x;
    std::vector<std::vector<double>> segment_force_y;

    /* parameters of forces */
    ParametersForRods parameter;
    double interaction_strength;
    double inversed_diameter_of_segment;

public:
    ForcesOnSegments2d(ParametersForRods parameter, double interaction_strength);
    ~ForcesOnSegments2d();
    std::pair<
        std::vector<std::vector<double>>,
        std::vector<std::vector<double>>
    > getForcesOnSegments2d() const;
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> getForcesOnRods2d() const;
    void calcForcesOnRods2d(const Rods& rods);
    void calcRodRodYukawaForces(const Rods& rods);
    void calcRodRodYukawaForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y);
    double nearestInPeriodic(double x1, double x2, double min_x, double max_x);   // 周期的境界により生じる像の内最短距離にあるものを探索
    void calcRodRodVicsekForces(const Rods& rods);
    void calcRodRodVicsekForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y);
    void calcRodRodPolarSinusoidalForces(const Rods& rods);
    void calcRodRodPolarSinusoidalForcesWithPeriodic(const Rods& rods, double min_x, double max_x, double min_y, double max_y);
    void ClearCalculatedForces();
};