#include <iostream>
#include <sstream>
#include <iomanip>
#include <numbers>
#include <string>
#include <utility>
#include <cmath>
#include <random>
#include "Rods.hpp"
#include "util.hpp"

Rods::Rods(ParametersForRods parameter) : parameter(parameter)
{
    double num_of_rods = parameter.num_of_rods;
    double num_of_segments = parameter.num_of_segments;

    std::cout << num_of_rods << " Rods are created" << std::endl;
    xg.resize(num_of_rods);
    yg.resize(num_of_rods);
    theta.resize(num_of_rods);
    sin.resize(num_of_rods);
    cos.resize(num_of_rods);

    segment_x.resize(num_of_rods);
    segment_y.resize(num_of_rods);
    for (auto &&i : segment_x)
    {
        i.resize(num_of_segments);
    }
    for (auto &&i : segment_y)
    {
        i.resize(num_of_segments);
    }

    /* temporary data */
    // velodity and angular velocity
    vx.resize(num_of_rods);
    vy.resize(num_of_rods);
    omega.resize(num_of_rods);
    // translational force and rotational force (torque)
    force_x.resize(num_of_rods);
    force_y.resize(num_of_rods);
    torque.resize(num_of_rods);
}

ActiveRods::ActiveRods(ParametersForActiveRods parameter) : Rods(parameter), parameter(parameter)
{
}

Rods::~Rods()
{
    std::cout << "Rods are deleted" << std::endl;
}

double Rods::initializeRodsOnSquareLattice()
{
    // set seed for random number
    std::mt19937 mt(1000);
    // indeterministic seed using device time
    // std::random_device rnd;
    // std::mt19937 mt(rnd());

    std::uniform_real_distribution<> uniform(- std::numbers::pi, std::numbers::pi);

    const int square_lattice_size = (int) std::ceil(std::sqrt(parameter.num_of_rods));

    for (size_t i = 0; i < xg.size(); i++) {
        xg.at(i) = parameter.length * (i % square_lattice_size);
        yg.at(i) = parameter.length * (i / square_lattice_size);
        theta.at(i) = uniform(mt);
        cos.at(i) = std::cos(theta.at(i));
        sin.at(i) = std::sin(theta.at(i));
    }

    calcSegmentsConformation();

    return parameter.length * square_lattice_size;
}

void Rods::calcSegmentsConformation()
{
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        for (size_t segment_j = 0; segment_j < segment_x.at(rod_i).size(); segment_j++) {
            double delta_length = (parameter.length - parameter.diameter_of_segment) * (0.5 - segment_j / (parameter.num_of_segments - 1.0));

            // position of segment i of rod i
            segment_x.at(rod_i).at(segment_j) = xg.at(rod_i) + delta_length * cos.at(rod_i);
            segment_y.at(rod_i).at(segment_j) = yg.at(rod_i) + delta_length * sin.at(rod_i);
        }
    }
}

void Rods::addForces(std::vector<double> force_x, std::vector<double> force_y, std::vector<double> torque)
{
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
            this->force_x.at(rod_i) += force_x.at(rod_i);
            this->force_y.at(rod_i) += force_y.at(rod_i);
            this->torque .at(rod_i) += torque .at(rod_i);
        }
}

void Rods::addForcesFromForcesOnSegments(std::vector<std::vector<double>> fx, std::vector<std::vector<double>> fy)
{
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        for (size_t segment_j = 0; segment_j < segment_x.at(rod_i).size(); segment_j++) {
            force_x.at(rod_i) += fx.at(rod_i).at(segment_j);
            force_y.at(rod_i) += fy.at(rod_i).at(segment_j);
            torque .at(rod_i) += (segment_x.at(rod_i).at(segment_j) - xg.at(rod_i)) * fy.at(rod_i).at(segment_j)
                               - (segment_y.at(rod_i).at(segment_j) - yg.at(rod_i)) * fx.at(rod_i).at(segment_j);
        }
    }
}

void Rods::calcVelocitiesFromForces()
{
    for (size_t i = 0; i < xg.size(); i++) {
        double force_parallel =   force_x.at(i) * cos.at(i) + force_y.at(i) * sin.at(i);
        double force_vertical = - force_x.at(i) * sin.at(i) + force_y.at(i) * cos.at(i);

        double velocity_parallel = parameter.fluidity_parallel * force_parallel;
        double velocity_vertical = parameter.fluidity_vertical * force_vertical;

        vx.at(i) = velocity_parallel * cos.at(i) - velocity_vertical * sin.at(i);
        vy.at(i) = velocity_parallel * sin.at(i) + velocity_vertical * cos.at(i);
        omega.at(i) = parameter.fluidity_rotation * torque.at(i);
    }
}

void ActiveRods::calcVelocitiesFromForces()
{
    for (size_t i = 0; i < xg.size(); i++) {
        double force_parallel =   force_x.at(i) * cos.at(i) + force_y.at(i) * sin.at(i);
        double force_vertical = - force_x.at(i) * sin.at(i) + force_y.at(i) * cos.at(i);

        double velocity_parallel = parameter.fluidity_parallel * (force_parallel + parameter.active_force);
        double velocity_vertical = parameter.fluidity_vertical * force_vertical;

        vx.at(i) = velocity_parallel * cos.at(i) - velocity_vertical * sin.at(i);
        vy.at(i) = velocity_parallel * sin.at(i) + velocity_vertical * cos.at(i);
        omega.at(i) = parameter.fluidity_rotation * torque.at(i);
    }
}

void Rods::updateRods()
{
    for (size_t i = 0; i < xg.size(); i++) {
        xg.at(i)    += vx.at(i);
        yg.at(i)    += vy.at(i);
        theta.at(i) += omega.at(i);

        if (theta.at(i) > 0) {
            theta.at(i) = std::fmod(theta.at(i) + M_PI, 2 * M_PI) - M_PI;
        } else {
            theta.at(i) = std::fmod(theta.at(i) - M_PI, 2 * M_PI) + M_PI;
        }

        cos.at(i) = std::cos(theta.at(i));
        sin.at(i) = std::sin(theta.at(i));
    }

    calcSegmentsConformation();
    resetTemporaryVariables();
}

void Rods::resetTemporaryVariables()
{
    std::fill(vx.begin()     , vx.end(),      0.);
    std::fill(vy.begin()     , vy.end(),      0.);
    std::fill(omega.begin()  , omega.end(),   0.);
    std::fill(force_x.begin(), force_x.end(), 0.);
    std::fill(force_y.begin(), force_y.end(), 0.);
    std::fill(torque.begin() , torque.end(),  0.);
}

void Rods::translate(double dx, double dy)
{
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        xg.at(rod_i) += dx;
        yg.at(rod_i) += dy;

        for (size_t segment_j = 0; segment_j < segment_x.at(rod_i).size(); segment_j++) {
            segment_x.at(rod_i).at(segment_j) += dx;
            segment_y.at(rod_i).at(segment_j) += dy;
        }
    }
}

void Rods::scalePosition(double scalar)
{
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        xg.at(rod_i) *= scalar;
        yg.at(rod_i) *= scalar;
    }

    calcSegmentsConformation();
}

void Rods::periodic(double inf_x, double sup_x, double inf_y, double sup_y)
{
    double period_x = sup_x - inf_x;
    double period_y = sup_y - inf_y;

    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        if (xg.at(rod_i) < inf_x) {
            xg.at(rod_i) += period_x;
        }
        else if (xg.at(rod_i) > sup_x) {
            xg.at(rod_i) -= period_x;
        }

        if (yg.at(rod_i) < inf_y) {
            yg.at(rod_i) += period_y;
        }
        else if (yg.at(rod_i) > sup_y) {
            yg.at(rod_i) -= period_y;
        }
    }

    calcSegmentsConformation();
}

std::pair<double, double> Rods::getPositionOfCenterOfMass(int rod_i) const
{
    return {xg.at(rod_i), yg.at(rod_i)};
}

std::pair<double, double> Rods::getPositionOfSegment(int rod_i, int segment_j) const
{
    return {segment_x.at(rod_i).at(segment_j), segment_y.at(rod_i).at(segment_j)};
}

double Rods::getAngle(int rod_i) const
{
    return theta.at(rod_i);
}

std::pair<double, double> Rods::getOrientation(int rod_i) const
{
    return {cos.at(rod_i), sin.at(rod_i)};
}

void Rods::writeData(std::string root_directory_of_data, std::string phase_name, int steps, int total_steps_digits) const
{
    std::ostringstream filename_number;
    filename_number << std::setw(total_steps_digits) << std::setfill('0') << steps;
    const std::string current_directory_of_data = root_directory_of_data + phase_name + "/";
    // output rods with vectors and segments with circles
    std::string filename_rods;
    filename_rods.reserve(100);
    filename_rods.append(current_directory_of_data)
        .append("rods/rods")
        .append(filename_number.str())
        .append(".dat");
    writeRodsData(filename_rods);
    std::string filename_segments;
    filename_segments.reserve(100);
    filename_segments.append(current_directory_of_data)
        .append("segments/segments")
        .append(filename_number.str())
        .append(".dat");
    writeRodsSegmentsData(filename_segments);
}

void Rods::writeData(std::string root_directory_of_data, std::string phase_name, int index, int steps, int total_steps_digits) const
{
    std::ostringstream filename_number;
    filename_number << std::setw(total_steps_digits) << std::setfill('0') << steps;
    const std::string current_directory_of_data = root_directory_of_data + phase_name + "/";
    // output rods with vectors and segments with circles
    std::string filename_rods;
    filename_rods.reserve(100);
    filename_rods.append(current_directory_of_data)
        .append("rods/rods")
        .append(std::to_string(index))
        .append("_")
        .append(filename_number.str())
        .append(".dat");
    writeRodsData(filename_rods);
    std::string filename_segments;
    filename_segments.reserve(100);
    filename_segments.append(current_directory_of_data)
        .append("segments/segments")
        .append(std::to_string(index))
        .append("_")
        .append(filename_number.str())
        .append(".dat");
    writeRodsSegmentsData(filename_segments);
}

void Rods::writeRodsData(std::string filename) const
{
    FILE *fp;
    fp = fopen(filename.c_str(),"w");
    FileOpenErrorCheck(fp, filename);

    for (size_t i = 0; i < xg.size(); i++) {
        double length_x = parameter.length * cos.at(i);
        double length_y = parameter.length * sin.at(i);
        double initial_point_x = xg.at(i) - length_x/2;
        double initial_point_y = yg.at(i) - length_y/2;
        double theta_pi_period = theta.at(i);
        if (theta_pi_period < 0) {   // color pi周期
            theta_pi_period += M_PI;
        }

        // label x y cosx siny theta
        fprintf(fp, "%4d %7.2f %7.2f %6.2f %6.2f %5.2f %10f %10f %10f %10f %10f\n",
            (int)i,
            initial_point_x , initial_point_y, length_x, length_y, theta_pi_period,
            xg.at(i), yg.at(i), vx.at(i), vy.at(i), omega.at(i)
        );
    }

    fclose(fp);
}

void Rods::writeRodsSegmentsData(std::string filename) const
{
    FILE *fp;
    fp = fopen(filename.c_str(),"w");
    FileOpenErrorCheck(fp, filename);

    for (size_t i = 0; i < xg.size(); i++) {
        double theta_pi_period = theta.at(i);
        if (theta_pi_period < 0) {   // color pi周期
            theta_pi_period += M_PI;
        }

        for (size_t j = 0; j < segment_x.at(i).size(); j++) {
            fprintf(fp, "%4d %2d %7.2f %7.2f %.2f %5.2f\n",
                (int)i, (int)j,
                segment_x.at(i).at(j), segment_y.at(i).at(j), parameter.diameter_of_segment / 2.0, theta_pi_period
            );
        }
    }

    fclose(fp);
}
