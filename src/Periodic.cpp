#include <iostream>
#include <nlohmann/json.hpp>

#include "Periodic.hpp"
#include "Rods.hpp"
#include "ParametersForRods.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"

using json = nlohmann::json;

Periodic::Periodic(ActiveRods &active_rods, ParametersForActiveRods &parameter_for_active_rods) : rods(active_rods), parameter(parameter_for_active_rods)
{
    std::cout << "-- Parameters for Active Rods in Simulation --" << std::endl;
    this->parameter.DisplayParameters();
}

Periodic::~Periodic()
{
}

void Periodic::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    const double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    const double y_max = phase_parameter_json["y_max"].template get<double>();

    parameter_json["time_scaled_interaction_strength"]
        = parameter_json["interaction_strength"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
    ForcesOnSegments2d rod_rod_vicsek_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        rod_rod_vicsek_forces.calcRodRodVicsekForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        auto [fx_rod_rod_vicsek, fy_rod_rod_vicsek, torque_rod_rod_vicsek] = rod_rod_vicsek_forces.getForcesOnRods2d();
        rod_rod_vicsek_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod_vicsek, fy_rod_rod_vicsek, torque_rod_rod_vicsek);

        rods.calcVelocitiesFromForces();

        // gifファイル作成のためのデータ出力開始
        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
        }
        // gifファイル作成のためのデータ出力終了
        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}
