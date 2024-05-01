#include <iostream>
#include <sstream>
#include <iomanip>
#include <numbers>
#include <string>
#include <utility>
#include <cmath>
#include <random>
#include <vector>
#include <numeric>
#include "Rods.hpp"
#include "util.hpp"

Rods::Rods(ParametersForRods parameter) : parameter(parameter)
{
    double num_of_rods = parameter.num_of_rods;
    double num_of_segments = parameter.num_of_segments;

    int num_of_intelligent_rods = static_cast<int>(num_of_rods * parameter.intelligent_rods_ratio);
    int num_of_vicsek_rods = num_of_rods - num_of_intelligent_rods;

    std::cout << num_of_rods << " Rods are created" << std::endl;
    std::cout << "Number of intelligent rods: " << num_of_intelligent_rods << std::endl;
    std::cout << "Number of Vicsek rods: " << num_of_vicsek_rods << std::endl;

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
    type.resize(num_of_rods);

    last_vx.resize(num_of_rods);
    last_vy.resize(num_of_rods);
    last_force_x.resize(num_of_rods);
    last_force_y.resize(num_of_rods);

    for (int i = 0; i < num_of_rods; ++i) {
        if (i < num_of_vicsek_rods) {
            type[i] = 0; // Vicsek rods
        } else {
            type[i] = 1; // Intelligent rods
        }
    }
}

ActiveRods::ActiveRods(ParametersForActiveRods parameter) : Rods(parameter), parameter(parameter)
{
}

IntelligentActiveRods::IntelligentActiveRods(ParametersForIntelligentActiveRods parameter) : Rods(parameter), parameter(parameter)
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

void Rods::addForceToRod(int rodIndex, double fx, double fy, double torque) {
    if (rodIndex >= 0 && rodIndex < xg.size()) { 
        this->force_x.at(rodIndex) += fx;
        this->force_y.at(rodIndex) += fy;
        this->torque.at(rodIndex) += torque;
    } else {
        std::cerr << "Error: Attempt to add force to a rod with invalid index " << rodIndex << std::endl;
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

    last_vx = vx;
    last_vy = vy;
    last_force_y = force_y;
    last_force_x = force_x;

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

double Rods::getAngleDifferenceWithinRange(int rod_i, double range_n) const
{
    double total_angle_difference = 0.0;
    int xg_size = static_cast<int>(xg.size());
    int count = 0;

    for (int rod_j = 0; rod_j < xg_size; rod_j++) {
        if (rod_j == rod_i) continue; // Skip the same rod

        double distance = sqrt(pow(xg[rod_i] - xg[rod_j], 2) + pow(yg[rod_i] - yg[rod_j], 2));

        if (distance <= range_n) {
            total_angle_difference += AngleDifference(rod_i, rod_j);
            count++;
        }
    }

    // Handle the case where no neighboring rods are found within the range
    if (count == 0) return 0.0;

    return total_angle_difference / count;
}

std::vector<double> Rods::getAllAngleDifferenceWithinRange(double range_n) const
{
    std::vector<double> allAngleDiff;

    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        double angleDiff = getAngleDifferenceWithinRange(rod_i, range_n);
        allAngleDiff.push_back(angleDiff);
    }

    return allAngleDiff;
}


double Rods::AngleDifference(int rod_i, int rod_j) const
{
    double rad_i = M_PI + theta.at(rod_i);
    double rad_j = M_PI + theta.at(rod_j);
    double d = fmodf(abs(rad_i - rad_j), (double)M_PI*2);
    double r = d > M_PI ? M_PI*2 - d : d;
    if ((rad_i - rad_j >= 0 && rad_i - rad_j <= M_PI) || (rad_i - rad_j <= -M_PI && rad_i - rad_j >= -M_PI*2)) 
        return r;
    return -r;
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
        fprintf(fp, "%4d %7.2f %7.2f %6.2f %6.2f %5.2f %10f %10f %10f %10f %10f %4d\n",
            (int)i,
            initial_point_x , initial_point_y, length_x, length_y, theta_pi_period,
            xg.at(i), yg.at(i), vx.at(i), vy.at(i), omega.at(i), type.at(i)
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
            fprintf(fp, "%4d %2d %7.2f %7.2f %.2f %5.2f %4d\n",
                (int)i, (int)j,
                segment_x.at(i).at(j), segment_y.at(i).at(j), parameter.diameter_of_segment / 2.0, theta_pi_period, type.at(i)
            );
        }
    }

    fclose(fp);
}

// Calculate active work done by an individual rod
double Rods::getActiveWorkByRod(int rod_i, double timeStep) 
{

    double displacement_x = this->last_vx.at(rod_i) * timeStep;
    double displacement_y = this->last_vy.at(rod_i) * timeStep;

    // This scale the reward higher
    const double reward_coefficient = 100000;

    return (this->last_force_x.at(rod_i) * displacement_x + this->last_force_y.at(rod_i) * displacement_y)*reward_coefficient;
}

// Calculate active work done by an individual rod
double Rods::getActiveWorkByRodWithinRange(size_t rod_i, double timeStep, double range_n) 
{
    double avg_active_work = 0.0;
    int count = 0;

    for (size_t rod_j = 0; rod_j < xg.size(); rod_j++) {
        // if (rod_j == rod_i) continue; // Skip the same rod

        double distance = sqrt(pow(xg[rod_i] - xg[rod_j], 2) + pow(yg[rod_i] - yg[rod_j], 2));

        if (distance <= range_n) {
            avg_active_work += getActiveWorkByRod(rod_j, timeStep);
            count++;
        }
    }

    // Handle the case where no neighboring rods are found within the range
    // Probably will not enter such case
    // if (count == 0) return 0.0;

    return avg_active_work / count;
}

// Calculate active work done by an individual rod within range
std::vector<double> Rods::getAllActiveWorkByRodWithinRange(double timeStep, double range_n) 
{
    std::vector<double> local_active_work;
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        double aw = getActiveWorkByRodWithinRange(rod_i, timeStep, range_n);
        local_active_work.push_back(aw);
    }
    return local_active_work;
}

// Calculate active work done by an individual rod
std::vector<double> Rods::getAllActiveWorkByRod(double timeStep) 
{
    std::vector<double> local_active_work;
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        double aw = getActiveWorkByRod(rod_i, timeStep);
        local_active_work.push_back(aw);
    }
    return local_active_work;
}

// Calculate local and global active work done by an individual rod
std::vector<double> Rods::getAllComplexActiveWorkByRod(double timeStep) 
{
    std::vector<double> active_work;
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        double aw = getActiveWorkByRod(rod_i, timeStep);
        active_work.push_back(aw);
    }

    double sum = std::accumulate(active_work.begin(), active_work.end(), 0.0);
    double average = sum / active_work.size();

    for (double& work : active_work) {
        work += average;
    }

    return active_work;
}

// Calculate total active work done by all rods
double Rods::getTotalActiveWork(double timeStep) 
{
    double totalWork = 0.0;

    for (size_t i = 0; i < xg.size(); ++i) {
        totalWork += this->getActiveWorkByRod(i, timeStep);
    }

    // Not sure if we want to get the average or the total active work
    return totalWork/xg.size();
}

// Helper function to check intersection between a ray and a rod segment/ and obstacle in the future
bool Rods::isIntersecting(double ray_x1, double ray_y1, double ray_x2, double ray_y2,
                    double segment_x, double segment_y, double segment_diameter) {
    // Calculate the closest point on the ray to the segment center
    double dx = ray_x2 - ray_x1;
    double dy = ray_y2 - ray_y1;
    double t = ((segment_x - ray_x1) * dx + (segment_y - ray_y1) * dy) / (dx * dx + dy * dy);
    double closest_x = ray_x1 + t * dx;
    double closest_y = ray_y1 + t * dy;

    // Check if this point is within the segment diameter
    double distance_to_segment = std::hypot(segment_x - closest_x, segment_y - closest_y);
    return distance_to_segment <= segment_diameter / 2.0;
}

// Get the 1D vision array of a rod
std::vector<double> Rods::get1DVisionArray(int rod_i, double r_v, int ray_count) {
    std::vector<double> vision_array(ray_count, 0.0);

    double x_pos = this->xg.at(rod_i);
    double y_pos = this->yg.at(rod_i);
    double rod_orientation = this->theta.at(rod_i);

    for (int i = 0; i < ray_count; ++i) {
        //Calculate ray direction based on rod orientation
        double ray_angle = rod_orientation + M_PI * (i - ray_count/2) / ray_count/2;
        double ray_dx = r_v * std::cos(ray_angle);
        double ray_dy = r_v * std::sin(ray_angle);

        for (size_t j = 0; j < xg.size(); ++j) {
            if (j == rod_i) continue; // Skip the same rod

            for (size_t segment = 0; segment < this->segment_x.at(j).size(); ++segment) {
                // Check if the ray intersects with segment of rod j
                if (isIntersecting(x_pos, y_pos, x_pos + ray_dx, y_pos + ray_dy,
                                   this->segment_x.at(j).at(segment),
                                   this->segment_y.at(j).at(segment),
                                   this->parameter.diameter_of_segment)) {
                    double distance_to_rod = std::hypot(this->segment_x.at(j).at(segment) - x_pos,
                                                        this->segment_y.at(j).at(segment) - y_pos);
                    double vision_value = 1.0 - distance_to_rod / r_v;
                    vision_array[i] = std::max(vision_array[i], vision_value); // Take the maximum value in case of multiple intersections
                    break; // Stop checking other segments of this rod if an intersection is found
                }
            }
        }
    }

    return vision_array;
}

// Get the 1D vision array of a rod
std::vector<std::vector<double>> Rods::getAll1DVisionArray(double r_v, int ray_count) {
    std::vector<std::vector<double>> all_vision_array;
    for (size_t rod_i = 0; rod_i < xg.size(); rod_i++) {
        std::vector<double> va = get1DVisionArray(rod_i, r_v, ray_count);
        all_vision_array.push_back(va);
    }
    return all_vision_array;
}