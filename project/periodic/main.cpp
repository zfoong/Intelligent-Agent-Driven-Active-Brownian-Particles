#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <random>
#include <nlohmann/json.hpp>
#include "Periodic.hpp"
#include "Rods.hpp"
#include "ParametersForRods.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"
using json = nlohmann::json;

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

    const std::string root_directory_of_data = "data/";

    // read a JSON file
    std::ifstream parameter_input(argv[1]);
    json parameter_json = json::parse(parameter_input);

    const double length_of_rods = parameter_json["length_of_rods"].template get<double>();
    const double diameter_of_segments = parameter_json["diameter_of_segments"].template get<double>();
    const int number_of_rods = parameter_json["number_of_rods"];
    const int square_lattice_size = (int) std::ceil(std::sqrt(number_of_rods));
    double linear_dimension = length_of_rods * square_lattice_size;

    const double time_interval_per_step = parameter_json["given_time_interval_per_step"].template get<double>() * std::sqrt(linear_dimension * linear_dimension / number_of_rods / diameter_of_segments / diameter_of_segments);
    parameter_json["time_interval_per_step"] = time_interval_per_step;
    const double steps_in_unit_time = 1. / time_interval_per_step;
    parameter_json["steps_in_unit_time"] = steps_in_unit_time;

    // 確認のための出力
    std::cout << "time_interval_per_step=" << time_interval_per_step << std::endl;
    std::cout << "steps_in_unit_time=" << steps_in_unit_time << std::endl;
    std::cout << "time interval per frame = " << time_interval_per_step * parameter_json["step_interval_for_output"].template get<int>() << std::endl;


    // time
    double dts, dte;
    dts = GetTime();

    const double total_times = parameter_json["phases"][0]["given_total_times"].template get<double>() * linear_dimension;
    const int total_steps = total_times * steps_in_unit_time;

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


    // parameters in phase
    parameter_json["phases"][0]["total_times"] = total_times;
    parameter_json["phases"][0]["total_steps"] = total_steps;
    parameter_json["phases"][0]["x_min"] = 0.;
    parameter_json["phases"][0]["x_max"] = linear_dimension;
    parameter_json["phases"][0]["y_min"] = 0.;
    parameter_json["phases"][0]["y_max"] = linear_dimension;

    WriteParametersJson(root_directory_of_data, "parameters_used.json", parameter_json);
    Periodic periodic(rods, parameter);
    periodic.Run(parameter_json, 0, root_directory_of_data, time_interval_per_step);
    WriteParametersJson(root_directory_of_data, "parameters_used.json", parameter_json);

    dte = GetTime();

    std::cerr << "elapsed time = " << dte-dts << "[sec], " << (dte-dts)/60 << "[min], " << (dte-dts)/3600 << "[hour]" << std::endl;

    return 0;
}
