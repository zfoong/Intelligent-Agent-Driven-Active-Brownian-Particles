#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

#include "ParametersForBoundary.hpp"
#include "Boundary.hpp"
#include "util.hpp"

Boundary::Boundary(
    ParametersForBoundary &parameter,
    int num_of_points
) : parameter(parameter), boundary_x(num_of_points), boundary_y(num_of_points)
{
}

int Boundary::getNumOfPoints() const
{
    return boundary_x.size();
}

void Boundary::setParameters(ParametersForBoundary const &parameter)
{
    this->parameter.setParameters(parameter);
}

void Boundary::writeData(std::string root_directory_of_data, std::string phase_name) const
{
    std::string filename;
    filename.reserve(100);
    filename.append(root_directory_of_data)
        .append(phase_name)
        .append("/boundary/boundary.dat");
    writePointsOnBoundary(filename);
}

void Boundary::writeData(std::string root_directory_of_data, std::string phase_name, int index) const
{
    std::string filename;
    filename.reserve(100);
    filename.append(root_directory_of_data)
        .append(phase_name)
        .append("/boundary/boundary")
        .append(std::to_string(index))
        .append(".dat");
    writePointsOnBoundary(filename);
}

void Boundary::writeData(std::string root_directory_of_data, std::string phase_name, int steps, int total_steps_digits) const
{
    std::ostringstream filename_number;
    filename_number << std::setw(total_steps_digits) << std::setfill('0') << steps;

    std::string filename;
    filename.reserve(100);
    filename.append(root_directory_of_data)
        .append(phase_name)
        .append("/boundary/boundary_")
        .append(filename_number.str())
        .append(".dat");
    writePointsOnBoundary(filename);
}

void Boundary::writeData(std::string root_directory_of_data, std::string phase_name, int index, int steps, int total_steps_digits) const
{
    std::ostringstream filename_number;
    filename_number << std::setw(total_steps_digits) << std::setfill('0') << steps;

    std::string filename;
    filename.reserve(100);
    filename.append(root_directory_of_data)
        .append(phase_name)
        .append("/boundary/boundary")
        .append(std::to_string(index))
        .append("_")
        .append(filename_number.str())
        .append(".dat");
    writePointsOnBoundary(filename);
}

void Boundary::writePointsOnBoundary(std::string filename) const
{
    FILE *fp;
    fp = fopen(filename.c_str(),"w");
    FileOpenErrorCheck(fp, filename);

    for (size_t i = 0; i < boundary_x.size(); i++) {
        fprintf(fp, "%.2f %.2f\n", boundary_x.at(i), boundary_y.at(i));
    }

    fclose(fp);
}

void Boundary::displayParameters() const
{
    parameter.displayParameters();
    std::cout << "num_of_points: " << boundary_x.size() << std::endl;
}
