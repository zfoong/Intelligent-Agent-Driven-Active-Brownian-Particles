#pragma once
#include <vector>
#include <string>

#include "ParametersForBoundary.hpp"

class Boundary
{
protected:
    ParametersForBoundary &parameter;
    std::vector<double> boundary_x, boundary_y; // points on boundary

public:
    Boundary(ParametersForBoundary &parameter, int num_of_points);
    virtual ~Boundary() = default;

    virtual int getNumOfPoints() const;
    virtual void setParameters(ParametersForBoundary const &parameter);

    virtual void calcPointsOnBoundary() = 0;
    virtual void writeData(std::string root_directory_of_data, std::string phase_name) const;
    virtual void writeData(std::string root_directory_of_data, std::string phase_name, int index) const;
    virtual void writeData(std::string root_directory_of_data, std::string phase_name, int steps, int total_steps_digits) const;
    virtual void writeData(std::string root_directory_of_data, std::string phase_name, int index, int steps, int total_steps_digits) const;
    virtual void writePointsOnBoundary(std::string filename) const;
    virtual void displayParameters() const;
};
