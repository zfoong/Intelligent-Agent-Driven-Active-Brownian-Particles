#pragma once

class ParametersForBoundary
{
public:
    ParametersForBoundary();
    virtual ~ParametersForBoundary();

    virtual void setParameters(ParametersForBoundary const &parameter);

    virtual void displayParameters() const = 0;
};
