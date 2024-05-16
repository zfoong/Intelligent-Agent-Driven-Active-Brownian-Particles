#include <iostream>
#include <cmath>
#include <utility>
#include <numbers>

#include "Boundary.hpp"
#include "ParametersForCircularBoundary.hpp"
#include "CircularBoundary.hpp"

CircularBoundary::CircularBoundary(
    ParametersForCircularBoundary &parameter,
    int num_of_points
) : Boundary(parameter, num_of_points),
    parameter(parameter),
    unit_circle_x(num_of_points),
    unit_circle_y(num_of_points)
{
    calcPointsOnUnitCircle();
    calcPointsOnBoundary();
    std::cout << "Circular Boundary is created" << std::endl;
}

CircularBoundary::~CircularBoundary()
{
    std::cout << "Circular Boundary is deleted" << std::endl;
}

double CircularBoundary::getRadius() const
{
    return parameter.getRadius();
}

std::pair<double, double> CircularBoundary::getPositionOfCenter() const
{
    return parameter.getPositionOfCenter();
}

void CircularBoundary::setRadius(double radius)
{
    this->parameter.setRadius(radius);
}

void CircularBoundary::setPositionOfCenter(double center_x, double center_y)
{
    this->parameter.setPositionOfCenter(center_x, center_y);
}

void CircularBoundary::setParameters(ParametersForCircularBoundary const &parameter)
{
    this->parameter.setParameters(parameter);
}

void CircularBoundary::calcPointsOnBoundary()
{
    const double radius = parameter.getRadius();
    auto [center_x, center_y] = parameter.getPositionOfCenter();
    for (size_t i = 0; i < boundary_x.size(); i++) {
        boundary_x.at(i) = radius * unit_circle_x.at(i) + center_x;
        boundary_y.at(i) = radius * unit_circle_y.at(i) + center_y;
    }
}

void CircularBoundary::calcPointsOnUnitCircle()
{
    constexpr double pi = std::numbers::pi;
    constexpr double tau = 2 * pi;

    const size_t num_of_points = unit_circle_x.size();
    // use (num_of_points - 1) to make start point and end point overlapped
    const double delta_theta = tau / (num_of_points - 1);
    for (size_t i = 0; i < num_of_points; i++) {
        double theta = i * delta_theta;
        unit_circle_x.at(i) = std::cos(theta);
        unit_circle_y.at(i) = std::sin(theta);
    }
}
