#include <iostream>
#include <utility>

#include "ParametersForCircularBoundary.hpp"

ParametersForCircularBoundary::ParametersForCircularBoundary(
    double radius,
    double center_x,
    double center_y
) : radius(radius),
    center_x(center_x),
    center_y(center_y)
{
}

ParametersForCircularBoundary::~ParametersForCircularBoundary()
{
}

double ParametersForCircularBoundary::getRadius() const
{
    return radius;
}

std::pair<double, double> ParametersForCircularBoundary::getPositionOfCenter() const
{
    return {center_x, center_y};
}

void ParametersForCircularBoundary::setRadius(double radius)
{
    this->radius = radius;
}

void ParametersForCircularBoundary::setPositionOfCenter(double center_x, double center_y)
{
    this->center_x = center_x;
    this->center_y = center_y;
}

void ParametersForCircularBoundary::setParameters(ParametersForCircularBoundary const &parameter)
{
    double radius = parameter.getRadius();
    auto [center_x, center_y] = parameter.getPositionOfCenter();
    this->radius = radius;
    this->center_x = center_x;
    this->center_y = center_y;
}

void ParametersForCircularBoundary::displayParameters() const
{
    std::cout << "radius: " << this->radius << std::endl;
    std::cout << "center_x: " << this->center_x << std::endl;
    std::cout << "center_y: " << this->center_y << std::endl;
}
