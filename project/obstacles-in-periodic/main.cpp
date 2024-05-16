#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <numbers>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "Periodic.hpp"
#include "ObstaclesInPeriodic.hpp"
#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "util.hpp"

#include <sys/time.h>
double GetTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double t = 0.0;
    t += (double)tv.tv_sec;
    t += (double)tv.tv_usec * 1.0e-6;
    return t;
}

int main(int argc, char** argv)
{
    // check arguments
    if ( argc != 2) {
        std::cout << "Usage   : $ ./a.out $parameter_file.json" << std::endl;
        std::cout << "example : $ ./a.out parameters.json" << std::endl;
        return -1;
    }



    // read a JSON file
    std::ifstream parameter_input(argv[1]);
    json parameter_json = json::parse(parameter_input);

    const std::string name = parameter_json["name"];
    const double length_of_rods = parameter_json["length_of_rods"].template get<double>();
    const double diameter_of_segments = parameter_json["diameter_of_segments"].template get<double>();
    const int number_of_rods = parameter_json["number_of_rods"];

    const int square_lattice_size = (int) std::ceil(std::sqrt(number_of_rods));
    const double initial_linear_dimension = length_of_rods * square_lattice_size;
    const double area_of_rods = number_of_rods * (diameter_of_segments * (length_of_rods - diameter_of_segments) + std::numbers::pi * diameter_of_segments * diameter_of_segments / 4);
    double area_of_obstacles = 0.;
    for (const auto &boundary : parameter_json["phases"][parameter_json["phases"].size() - 1]["boundaries"]) {
        const double boundary_radius = boundary["radius"].template get<double>();
        area_of_obstacles += std::numbers::pi * boundary_radius * boundary_radius;
    }
    const double area_fraction = parameter_json["area_fraction"].template get<double>();
    const double final_linear_dimension = std::sqrt((area_of_rods / area_fraction) + area_of_obstacles);
    parameter_json["initial_linear_dimension"] = initial_linear_dimension;
    parameter_json["final_linear_dimension"] = final_linear_dimension;

    double linear_dimension = std::max(initial_linear_dimension, final_linear_dimension);

    const double time_interval_per_step = parameter_json["given_time_interval_per_step"].template get<double>() * std::sqrt(linear_dimension * linear_dimension / number_of_rods / diameter_of_segments / diameter_of_segments);
    parameter_json["time_interval_per_step"] = time_interval_per_step;
    const double steps_in_unit_time = 1. / time_interval_per_step;
    parameter_json["steps_in_unit_time"] = steps_in_unit_time;

    // 確認のための出力
    std::cout << "time_interval_per_step=" << time_interval_per_step << std::endl;
    std::cout << "steps_in_unit_time=" << steps_in_unit_time << std::endl;
    std::cout << "time interval per frame = " << time_interval_per_step * parameter_json["step_interval_for_output"].template get<int>() << std::endl;

    // define saved data directory
    const std::string root_directory_of_data = "data/";
    const std::string directory = root_directory_of_data + name + "/";

    // time
    double dts, dte;
    dts = GetTime();

    ParametersForActiveRods parameter(parameter_json);
    std::cout << "-- Parameters for Rods in Real Time --" << std::endl;
    parameter.DisplayParameters();
    parameter.ConvertTimeScale(time_interval_per_step);
    std::cout << "-- Parameters for Rods in Simulation Unit Time --" << std::endl;
    parameter.DisplayParameters();
    ActiveRods rods(parameter);
    rods.initializeRodsOnSquareLattice();
    rods.translate(length_of_rods / 2, length_of_rods / 2);
    rods.periodic(0., linear_dimension, 0., linear_dimension);

    {
        const int phase_index = 0;
        const double total_times = parameter_json["phases"][phase_index]["given_total_times"].template get<double>();
        const int total_steps = total_times * steps_in_unit_time;

        // parameters in phase
        parameter_json["phases"][phase_index]["total_times"] = total_times;
        parameter_json["phases"][phase_index]["total_steps"] = total_steps;
        parameter_json["phases"][phase_index]["x_min"] = 0.;
        parameter_json["phases"][phase_index]["x_max"] = initial_linear_dimension;
        parameter_json["phases"][phase_index]["y_min"] = 0.;
        parameter_json["phases"][phase_index]["y_max"] = initial_linear_dimension;

        json boundaries_json = parameter_json["phases"][phase_index]["boundaries"];
        std::vector<ParametersForCircularBoundary> parameters_for_boundary;
        parameters_for_boundary.reserve(boundaries_json.size());
        for (size_t i = 0; i < boundaries_json.size(); i++) {
            const double radius = boundaries_json[i]["radius"].template get<double>();
            const double center_x = boundaries_json[i]["center"]["x"].template get<double>();
            const double center_y = boundaries_json[i]["center"]["y"].template get<double>();
            if (boundaries_json[i]["center"]["relative"]) {
                parameters_for_boundary.emplace_back(
                    radius,
                    initial_linear_dimension * center_x,
                    initial_linear_dimension * center_y
                );
            } else {
                parameters_for_boundary.emplace_back(
                    radius,
                    center_x,
                    center_y
                );
            }
        }
        const double final_interaction_strength_between_rod_obstacle = parameter_json["interaction_strength_between_rod_obstacle"].template get<double>();
        const double ratio = 0.001;
        const double initial_interaction_strength_between_rod_obstacle = final_interaction_strength_between_rod_obstacle * ratio;
        parameter_json["initial_interaction_strength_between_rod_obstacle"] = initial_interaction_strength_between_rod_obstacle;
        parameter_json["final_interaction_strength_between_rod_obstacle"] = final_interaction_strength_between_rod_obstacle;

        WriteParametersJson(directory, "parameters_used.json", parameter_json);
        ObstaclesSteepeningInPeriodic simulation(rods, parameter, parameters_for_boundary);
        simulation.Run(parameter_json, phase_index, directory, time_interval_per_step, initial_interaction_strength_between_rod_obstacle * ratio, final_interaction_strength_between_rod_obstacle);
        WriteParametersJson(directory, "parameters_used.json", parameter_json);
    }

    {
        const int phase_index = 1;
        const double total_times = parameter_json["phases"][phase_index]["given_total_times"].template get<double>() * (linear_dimension - final_linear_dimension);
        const int total_steps = total_times * steps_in_unit_time;

        // parameters in phase
        parameter_json["phases"][phase_index]["total_times"] = total_times;
        parameter_json["phases"][phase_index]["total_steps"] = total_steps;
        parameter_json["phases"][phase_index]["x_min"] = 0.;
        parameter_json["phases"][phase_index]["x_max"] = linear_dimension;
        parameter_json["phases"][phase_index]["y_min"] = 0.;
        parameter_json["phases"][phase_index]["y_max"] = linear_dimension;

        const bool shouldBeCompressed = final_linear_dimension < initial_linear_dimension;
        if (shouldBeCompressed) {
            json boundaries_json = parameter_json["phases"][phase_index]["boundaries"];
            std::vector<ParametersForCircularBoundary> parameters_for_boundary;
            parameters_for_boundary.reserve(boundaries_json.size());
            for (size_t i = 0; i < boundaries_json.size(); i++) {
                const double radius = boundaries_json[i]["radius"].template get<double>();
                const double center_x = boundaries_json[i]["center"]["x"].template get<double>();
                const double center_y = boundaries_json[i]["center"]["y"].template get<double>();
                if (boundaries_json[i]["center"]["relative"]) {
                    parameters_for_boundary.emplace_back(
                        radius,
                        initial_linear_dimension * center_x,
                        initial_linear_dimension * center_y
                    );
                } else {
                    parameters_for_boundary.emplace_back(
                        radius,
                        center_x,
                        center_y
                    );
                }
            }

            WriteParametersJson(directory, "parameters_used.json", parameter_json);
            ObstaclesInPeriodicCompression simulation(rods, parameter, parameters_for_boundary);
            simulation.Run(parameter_json, phase_index, directory, time_interval_per_step, initial_linear_dimension, final_linear_dimension);
            WriteParametersJson(directory, "parameters_used.json", parameter_json);
            linear_dimension = final_linear_dimension;
        }
    }

    {
        const int phase_index = 2;
        const double total_times = parameter_json["phases"][phase_index]["given_total_times"].template get<double>() * linear_dimension;
        const int total_steps = total_times * steps_in_unit_time;

        // parameters in phase
        parameter_json["phases"][phase_index]["total_times"] = total_times;
        parameter_json["phases"][phase_index]["total_steps"] = total_steps;
        parameter_json["phases"][phase_index]["x_min"] = 0.;
        parameter_json["phases"][phase_index]["x_max"] = linear_dimension;
        parameter_json["phases"][phase_index]["y_min"] = 0.;
        parameter_json["phases"][phase_index]["y_max"] = linear_dimension;

        json boundaries_json = parameter_json["phases"][phase_index]["boundaries"];
        std::vector<ParametersForCircularBoundary> parameters_for_boundary;
        parameters_for_boundary.reserve(boundaries_json.size());
        for (size_t i = 0; i < boundaries_json.size(); i++) {
            const double radius = boundaries_json[i]["radius"].template get<double>();
            const double center_x = boundaries_json[i]["center"]["x"].template get<double>();
            const double center_y = boundaries_json[i]["center"]["y"].template get<double>();
            if (boundaries_json[i]["center"]["relative"]) {
                parameters_for_boundary.emplace_back(
                    radius,
                    linear_dimension * center_x,
                    linear_dimension * center_y
                );
            } else {
                parameters_for_boundary.emplace_back(
                    radius,
                    center_x,
                    center_y
                );
            }
        }

        WriteParametersJson(directory, "parameters_used.json", parameter_json);
        ObstaclesInPeriodic simulation(rods, parameter, parameters_for_boundary);
        simulation.Run(parameter_json, phase_index, directory, time_interval_per_step);
        WriteParametersJson(directory, "parameters_used.json", parameter_json);
    }

    dte = GetTime();

    std::cerr << "elapsed time = " << dte-dts << "[sec], " << (dte-dts)/60 << "[min], " << (dte-dts)/3600 << "[hour]" << std::endl;

    return 0;
}
