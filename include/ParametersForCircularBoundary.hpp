#pragma once
#include <utility>

#include "ParametersForBoundary.hpp"

class ParametersForCircularBoundary : public ParametersForBoundary
{
private:
    // re-declare setParameters() to avoid hiding the base class's setParameters()
    using ParametersForBoundary::setParameters;

protected:
    double radius;
    double center_x, center_y;

public:
    ParametersForCircularBoundary(double radius, double center_x, double center_y);
    ~ParametersForCircularBoundary();

    double getRadius() const;
    std::pair<double, double> getPositionOfCenter() const;
    void setRadius(double radius);
    void setPositionOfCenter(double center_x, double center_y);
    void setParameters(ParametersForCircularBoundary const &parameter);

    void displayParameters() const override;
};
