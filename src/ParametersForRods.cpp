#include <iostream>
#include <cmath>
#include <tuple>
#include <nlohmann/json.hpp>

#include "ParametersForRods.hpp"

using json = nlohmann::json;

ParametersForRods::ParametersForRods()
{
}

ParametersForRods::ParametersForRods(
    double length,
    double diameter_of_segment,
    double viscosity,
    int num_of_rods,
    int num_of_segments
)
{
    this->length = length;
    this->diameter_of_segment = diameter_of_segment;
    this->aspect_ratio = this->length / this->diameter_of_segment;
    this->num_of_rods = num_of_rods;
    this->num_of_segments = num_of_segments;

    std::tie(
        this->viscosity_parallel,
        this->viscosity_vertical,
        this->viscosity_rotation
    ) = CalcViscosities(viscosity, this->aspect_ratio);

    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );
}

ParametersForRods::ParametersForRods(
    double length,
    double diameter_of_segment,
    double viscosity,
    int num_of_rods
)
{
    this->length = length;
    this->diameter_of_segment = diameter_of_segment;
    this->aspect_ratio = this->length / this->diameter_of_segment;
    this->num_of_rods = num_of_rods;
    this->num_of_segments = CalcNumberOfSegment(this->aspect_ratio);

    std::tie(
        this->viscosity_parallel,
        this->viscosity_vertical,
        this->viscosity_rotation
    ) = CalcViscosities(viscosity, this->aspect_ratio);

    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );
}

ParametersForRods::ParametersForRods(json parameter_json)
{
    this->length = parameter_json["length_of_rods"].template get<double>();
    this->diameter_of_segment = parameter_json["diameter_of_segments"].template get<double>();
    this->aspect_ratio = this->length / this->diameter_of_segment;
    this->num_of_rods = parameter_json["number_of_rods"].template get<int>();
    this->num_of_segments = CalcNumberOfSegment(this->aspect_ratio);

    double viscosity = parameter_json["viscosity"].template get<double>();

    std::tie(
        this->viscosity_parallel,
        this->viscosity_vertical,
        this->viscosity_rotation
    ) = CalcViscosities(viscosity, this->aspect_ratio);

    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );
}

ParametersForRods::~ParametersForRods()
{
}

ParametersForActiveRods::ParametersForActiveRods(ParametersForRods parameter, double active_force)
{
    this->length = parameter.length;
    this->diameter_of_segment = parameter.diameter_of_segment;
    this->aspect_ratio = parameter.aspect_ratio;
    this->num_of_rods = parameter.num_of_rods;
    this->num_of_segments = parameter.num_of_segments;
    this->viscosity_parallel = parameter.viscosity_parallel;
    this->viscosity_vertical = parameter.viscosity_vertical;
    this->viscosity_rotation = parameter.viscosity_rotation;
    this->fluidity_parallel = parameter.fluidity_parallel;
    this->fluidity_vertical = parameter.fluidity_vertical;
    this->fluidity_rotation = parameter.fluidity_rotation;
    this->active_force = active_force;
    this->self_propelling_velocity = this->active_force * this->fluidity_parallel;
}

ParametersForActiveRods::ParametersForActiveRods(json parameter_json)
{
    this->length = parameter_json["length_of_rods"].template get<double>();
    this->diameter_of_segment = parameter_json["diameter_of_segments"].template get<double>();
    this->aspect_ratio = this->length / this->diameter_of_segment;
    this->num_of_rods = parameter_json["number_of_rods"].template get<int>();
    this->num_of_segments = CalcNumberOfSegment(this->aspect_ratio);

    double viscosity = parameter_json["viscosity"].template get<double>();

    std::tie(
        this->viscosity_parallel,
        this->viscosity_vertical,
        this->viscosity_rotation
    ) = CalcViscosities(viscosity, this->aspect_ratio);

    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );

    this->active_force = parameter_json["self_propelling_force"].template get<double>();
    this->self_propelling_velocity = this->active_force * this->fluidity_parallel;
}

ParametersForIntelligentActiveRods::ParametersForIntelligentActiveRods(ParametersForRods parameter, double active_force)
{
    this->length = parameter.length;
    this->diameter_of_segment = parameter.diameter_of_segment;
    this->aspect_ratio = parameter.aspect_ratio;
    this->num_of_rods = parameter.num_of_rods;
    this->num_of_segments = parameter.num_of_segments;
    this->viscosity_parallel = parameter.viscosity_parallel;
    this->viscosity_vertical = parameter.viscosity_vertical;
    this->viscosity_rotation = parameter.viscosity_rotation;
    this->fluidity_parallel = parameter.fluidity_parallel;
    this->fluidity_vertical = parameter.fluidity_vertical;
    this->fluidity_rotation = parameter.fluidity_rotation;
    this->active_force = active_force;
    this->self_propelling_velocity = this->active_force * this->fluidity_parallel;
}

ParametersForIntelligentActiveRods::ParametersForIntelligentActiveRods(json parameter_json)
{
    this->length = parameter_json["length_of_rods"].template get<double>();
    this->diameter_of_segment = parameter_json["diameter_of_segments"].template get<double>();
    this->aspect_ratio = this->length / this->diameter_of_segment;
    this->num_of_rods = parameter_json["number_of_rods"].template get<int>();
    this->num_of_segments = CalcNumberOfSegment(this->aspect_ratio);

    double viscosity = parameter_json["viscosity"].template get<double>();

    std::tie(
        this->viscosity_parallel,
        this->viscosity_vertical,
        this->viscosity_rotation
    ) = CalcViscosities(viscosity, this->aspect_ratio);

    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );

    this->active_force = parameter_json["self_propelling_force"].template get<double>();
    this->self_propelling_velocity = this->active_force * this->fluidity_parallel;
}

// aspect ratioに従ってsegment数を決定
int ParametersForRods::CalcNumberOfSegment(double aspect_ratio)
{
    if (aspect_ratio > 3) {
        return (int)std::round(9.0 / 8.0 * aspect_ratio);
    } else if (aspect_ratio > 1){
        return 3;
    } else {
        return 1;
    }
}

// parallel, vertical, rotationalの摩擦係数の計算
std::tuple<double, double, double> ParametersForRods::CalcViscosities(double viscosity, double aspect_ratio) const
{
    const double viscosity_parallel = viscosity * 2 * M_PI / (std::log(aspect_ratio) - 0.207 + 0.980 / aspect_ratio - 0.133 / aspect_ratio / aspect_ratio);
    const double viscosity_vertical = viscosity * 4 * M_PI / (std::log(aspect_ratio) + 0.839 + 0.185 / aspect_ratio + 0.233 / aspect_ratio / aspect_ratio);
    const double viscosity_rotation = viscosity * aspect_ratio * aspect_ratio * M_PI / 3 / (std::log(aspect_ratio) - 0.662 + 0.917 / aspect_ratio - 0.050 / aspect_ratio / aspect_ratio);

    return {viscosity_parallel, viscosity_vertical, viscosity_rotation};
}

// parallel, vertical, rotationalの流動性の計算
void ParametersForRods::SetFluidities(double fluidity_parallel, double fluidity_vertical, double fluidity_rotation)
{
    this->fluidity_parallel = fluidity_parallel;
    this->fluidity_vertical = fluidity_vertical;
    this->fluidity_rotation = fluidity_rotation;
}

void ParametersForRods::ConvertTimeScale(double scalar)
{
    this->viscosity_parallel *= scalar;
    this->viscosity_vertical *= scalar;
    this->viscosity_rotation *= scalar;
    SetFluidities(
        1. / this->viscosity_parallel,
        1. / this->viscosity_vertical,
        1. / this->viscosity_rotation
    );
}

void ParametersForActiveRods::ConvertTimeScale(double scalar)
{
    ParametersForRods::ConvertTimeScale(scalar);
    this->active_force *= scalar * scalar;
    this->self_propelling_velocity = this->active_force * this->fluidity_parallel;
}

void ParametersForRods::DisplayParameters() const
{
    std::cout << "length of rods = " << this->length << std::endl;
    std::cout << "diameter of segments = " << this->diameter_of_segment << std::endl;
    std::cout << "aspect ratio = " << this->aspect_ratio << std::endl;
    std::cout << "number of rods = " << this->num_of_rods << std::endl;
    std::cout << "number of segments = " << this->num_of_segments << std::endl;
    std::cout << "parallel viscosity = " << this->viscosity_parallel << std::endl;
    std::cout << "vertical viscosity = " << this->viscosity_vertical << std::endl;
    std::cout << "rotational viscosity = " << this->viscosity_rotation << std::endl;
    std::cout << "parallel fluidity = " << this->fluidity_parallel << std::endl;
    std::cout << "vertical fluidity = " << this->fluidity_vertical << std::endl;
    std::cout << "rotational fluidity = " << this->fluidity_rotation << std::endl;
}

void ParametersForActiveRods::DisplayParameters() const
{
    ParametersForRods::DisplayParameters();
    std::cout << "active force = " << this->active_force << std::endl;
    std::cout << "self-propelling velocity = " << this->self_propelling_velocity << std::endl;
}

void ParametersForIntelligentActiveRods::DisplayParameters() const
{
    ParametersForRods::DisplayParameters();
    std::cout << "active force = " << this->active_force << std::endl;
    std::cout << "self-propelling velocity = " << this->self_propelling_velocity << std::endl;
}