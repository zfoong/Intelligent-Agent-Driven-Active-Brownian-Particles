#pragma once
#include <vector>
#include <string>
#include <utility>
#include "ParametersForRods.hpp"

class Rods
{
protected:
    std::vector<double> xg, yg;     // center of mass
    std::vector<double> theta;      // angle on 2D plane
    std::vector<double> cos, sin;   // orientation calculated from theta

    std::vector<std::vector<double>> segment_x;
    std::vector<std::vector<double>> segment_y;

    /* temporary data */
    std::vector<double> vx, vy, omega;      // velocities of rods
    std::vector<double> force_x, force_y;   // forces on center of mass
    std::vector<double> torque;             // torque around z-axis through center of mass


    /* parameters of rods */
    const ParametersForRods parameter;

public:
    Rods(ParametersForRods parameter);
    ~Rods();
    double initializeRodsOnSquareLattice();
    void calcSegmentsConformation();
    void addForces(std::vector<double> force_x, std::vector<double> force_y, std::vector<double> torque);
    void addForcesFromForcesOnSegments(std::vector<std::vector<double>> fx, std::vector<std::vector<double>> fy);
    void calcVelocitiesFromForces();
    void updateRods();
    void resetTemporaryVariables();
    void translate(double dx, double dy);
    void scalePosition(double scalar);
    void periodic(double inf_x, double sup_x, double inf_y, double sup_y);
    std::pair<double, double> getPositionOfCenterOfMass(int rod_i) const;
    std::pair<double, double> getPositionOfSegment(int rod_i, int segment_j) const;
    double getAngle(int rod_i) const;
    std::pair<double, double> getOrientation(int rod_i) const;
    void writeData(std::string root_directory_of_data, std::string phase_name, int steps, int digits) const;
    void writeData(std::string root_directory_of_data, std::string phase_name, int index, int steps, int digits) const;
    void writeRodsData(std::string filename) const;
    void writeRodsSegmentsData(std::string filename) const;
};

class ActiveRods : public Rods
{
private:
    /* parameters of active rods */
    const ParametersForActiveRods parameter;

public:
    ActiveRods(ParametersForActiveRods parameter);
    void calcVelocitiesFromForces();
};