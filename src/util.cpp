#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <nlohmann/json.hpp>
#include "util.hpp"

using json = nlohmann::json;

double CalcSquaredDistance(double dx, double dy)
{
    return dx * dx + dy * dy;
}

void DisplayProgress(int steps, int total_steps)
{
    std::string progress_status;
    progress_status.reserve(100);
    progress_status.append("Progress is ")
        .append(std::to_string(steps / (total_steps / 100))).append("%: ")
        .append(std::to_string(steps)).append(" steps in ")
        .append(std::to_string(total_steps)).append(" total steps");
    std::cout << progress_status << std::endl;
}

void FileOpenErrorCheck(FILE *fp, std::string filename)
{
    if (fp == NULL) {   // エラーチェック
        std::cerr << "error: cannot open " + filename << std::endl;
        exit(-1);
    }
}

void WriteParametersJson(std::string dirname, std::string filename, json parameter_json)
{
    // write prettified JSON to another file
    std::string fullpath_filename = dirname + filename;
    std::ofstream parameter_output(fullpath_filename);
    parameter_output << std::setw(2) << parameter_json << std::endl;
}
