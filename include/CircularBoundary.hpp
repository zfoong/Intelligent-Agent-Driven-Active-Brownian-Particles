#pragma once
#include <vector>
#include <utility>

#include "Boundary.hpp"
#include "ParametersForCircularBoundary.hpp"

class CircularBoundary : public Boundary
{
private:
    // re-declare methods to avoid hiding the base class's methods
    using Boundary::setParameters;

protected:
    ParametersForCircularBoundary &parameter;
    std::vector<double> unit_circle_x, unit_circle_y;   // points on unit circle

public:
    CircularBoundary(ParametersForCircularBoundary &parameter, int num_of_points);
    virtual ~CircularBoundary() override;

    double getRadius() const;
    std::pair<double, double> getPositionOfCenter() const;
    void setRadius(double radius);
    void setPositionOfCenter(double center_x, double center_y);
    virtual void setParameters(ParametersForCircularBoundary const &parameter);

    virtual void calcPointsOnBoundary() override;
    void calcPointsOnUnitCircle();
};
