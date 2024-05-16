#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "ObstaclesInPeriodic.hpp"
#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForForces.hpp"
#include "RodsCircularObstacleCappedLennardJonesForces.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"

ObstaclesSteepeningInPeriodic::ObstaclesSteepeningInPeriodic(
    Rods &rods,
    ParametersForRods &parameter_for_rods,
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles
) : rods(rods),
    parameter(parameter_for_rods),
    parameters_for_obstacles(parameters_for_obstacles)
{
    obstacles.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.emplace_back(parameters_for_obstacles.at(i), 1000);
    }

    std::cout << "-- Parameters for Rods in Obstacles Steepening --" << std::endl;
    this->parameter.DisplayParameters();

    std::cout << "-- Parameters for Obstacles in Obstacles Steepening --" << std::endl;
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        std::cout << "Obstacle " << i << ":" << std::endl;
        parameters_for_obstacles.at(i).displayParameters();
    }
}

ObstaclesSteepeningInPeriodic::~ObstaclesSteepeningInPeriodic()
{
}

void ObstaclesSteepeningInPeriodic::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_interaction_strength, double final_interaction_strength)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    double y_max = phase_parameter_json["y_max"].template get<double>();

    // output obstacle shape as discrete points
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.at(i).writeData(root_directory_of_data, phase_name, i);
    }

    // declare rod-rod forces
    parameter_json["time_scaled_interaction_strength_between_rod_rod"]
        = parameter_json["interaction_strength_between_rod_rod"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength_between_rod_rod"].template get<double>());

    // declare rod-obstacle forces
    parameter_json["time_scaled_initial_interaction_strength_between_rod_obstacle"]
        = initial_interaction_strength / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    parameter_json["time_scaled_final_interaction_strength_between_rod_obstacle"]
        = final_interaction_strength / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    // assume final_strength > initial_strength
    const double ratio = std::pow(final_interaction_strength / initial_interaction_strength, 1. / total_steps);

    std::vector<ParametersForForces> parameters_for_forces;
    parameters_for_forces.reserve(parameters_for_obstacles.size());
    std::vector<RodsCircularObstacleCappedLennardJonesForces> rod_obstacle_forces;
    rod_obstacle_forces.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        parameters_for_forces.emplace_back(parameter_json["time_scaled_initial_interaction_strength_between_rod_obstacle"].template get<double>());
        rod_obstacle_forces.emplace_back(parameter, parameters_for_obstacles.at(i), parameters_for_forces.at(i));
    }

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        // compute rod-rod forces
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        // compute rod-obstacle forces
        for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
            rod_obstacle_forces.at(i).compute(rods);
            auto [fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle] = rod_obstacle_forces.at(i).getForcesOnRods();
            rod_obstacle_forces.at(i).clear();
            rods.addForces(fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle);
        }

        // compute velocity from total forces
        rods.calcVelocitiesFromForces();

        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
        }

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        // update for next step
        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        // steepening obstacles
        for (size_t i = 0; i < rod_obstacle_forces.size(); i++) {
            const double strength = rod_obstacle_forces.at(i).getInteractionStrength();
            rod_obstacle_forces.at(i).setInteractionStrength(strength * ratio);
        }
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}
