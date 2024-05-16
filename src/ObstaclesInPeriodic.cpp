#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <deque>
#include <numeric>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "ObstaclesInPeriodic.hpp"
#include "ParametersForRods.hpp"
#include "Rods.hpp"
#include "ParametersForForces.hpp"
#include "RodsCircularObstacleYukawaForces.hpp"
#include "RodsCircularObstacleCappedLennardJonesForces.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"
#include "RLAgent.cpp"

ObstaclesInPeriodic::ObstaclesInPeriodic(
    ActiveRods &active_rods,
    ParametersForActiveRods &parameter_for_active_rods,
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles
) : Periodic(active_rods, parameter_for_active_rods),
    parameters_for_obstacles(parameters_for_obstacles)
{
    obstacles.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.emplace_back(parameters_for_obstacles.at(i), 1000);
    }

    std::cout << "-- Parameters for Active Rods in Simulation --" << std::endl;
    this->parameter.DisplayParameters();

    std::cout << "-- Parameters for Obstacles in Simulation --" << std::endl;
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        std::cout << "Obstacle " << i << ":" << std::endl;
        parameters_for_obstacles.at(i).displayParameters();
    }
}

ObstaclesInPeriodic::~ObstaclesInPeriodic()
{
}

void ObstaclesInPeriodic::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    const double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    const double y_max = phase_parameter_json["y_max"].template get<double>();

    // output obstacle shape as discrete points
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.at(i).writeData(root_directory_of_data, phase_name, i);
    }

    // RL parameters
    int update_freq = parameter_json["update_freq"].template get<int>();                     // perform backprop in every n time step
    int save_model_freq = parameter_json["save_model_freq"].template get<int>();             // save model in every n time step
    int action_std_decay_freq = parameter_json["action_std_decay_freq"].template get<int>(); // action_std decay frequency in every n timesteps
    double action_std = parameter_json["action_std"].template get<double>();                // starting std for action distribution (Multivariate Normal)
    double action_std_decay_rate = parameter_json["action_std_decay_rate"].template get<double>(); // linearly decay action_std
    double min_action_std = parameter_json["min_action_std"].template get<double>();        // minimum action_std (stop decay after action_std <= min_action_std)
    int K_epochs = parameter_json["K_epochs"].template get<int>();                          // update policy for K epochs in one PPO update
    double eps_clip = parameter_json["eps_clip"].template get<double>();                    // clip parameter for PPO
    double gamma = parameter_json["gamma"].template get<double>();                          // discount factor
    double lr_actor = parameter_json["lr_actor"].template get<double>();                    // learning rate for actor network
    double lr_critic = parameter_json["lr_critic"].template get<double>();                  // learning rate for critic network
    double reward_accum_delta = parameter_json["reward_accum_delta"].template get<double>(); // reward accumulate for n time step
    double range_neighbor = parameter_json["range_neighbor"].template get<double>();        // perception range of rods
    double intelligent_rods_ratio = parameter_json["intelligent_rods_ratio"].template get<double>(); // ratio of intelligent rods
    double vision_ray_count = 16;
    double r_v = 10;

    const int num_of_intelligent_rods = static_cast<int>(parameter.num_of_rods * intelligent_rods_ratio);
    const int num_of_vicsek_rods = parameter.num_of_rods - num_of_intelligent_rods;

    // declare rod-rod forces
    parameter_json["time_scaled_interaction_strength_between_rod_rod"]
        = parameter_json["interaction_strength_between_rod_rod"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength_between_rod_rod"].template get<double>());

    // declare rod-rod forces
    parameter_json["time_scaled_interaction_strength_between_rod_rod_rl"]
        = parameter_json["interaction_strength_between_rod_rod_rl"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_rl_agent_forces(parameter, parameter_json["time_scaled_interaction_strength_between_rod_rod_rl"].template get<double>());

    // declare rod-obstacle forces
    parameter_json["time_scaled_interaction_strength_between_rod_obstacle"]
        = parameter_json["interaction_strength_between_rod_obstacle"].template get<double>() / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    std::vector<ParametersForForces> parameters_for_forces;
    parameters_for_forces.reserve(parameters_for_obstacles.size());
    std::vector<RodsCircularObstacleYukawaForces> rod_obstacle_forces;
    rod_obstacle_forces.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        parameters_for_forces.emplace_back(parameter_json["time_scaled_interaction_strength_between_rod_obstacle"].template get<double>(), parameter.diameter_of_segment);
        rod_obstacle_forces.emplace_back(parameter, parameters_for_obstacles.at(i), parameters_for_forces.at(i));
    }

    // Initialize RL agent
    RLAgent rl_agent(vision_ray_count, 1, lr_actor, lr_critic, gamma, K_epochs, eps_clip, action_std);

    // RL: Get current state of the environment
    std::vector<std::vector<double>> state = rods.getAll1DVisionArray(r_v, vision_ray_count);

    const int reward_history_length = 10;
    std::deque<std::vector<double>> reward_history;

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        // RL: Agent decides action based on current state
        std::vector<double> actions = rl_agent.selectAllAction(state);

        // RL: Apply artificial force to the intelligent active rods
        rod_rod_rl_agent_forces.calcRLAgentArtificialForcesWithPeriodic(rods, actions, x_min, x_max, y_min, y_max);
        auto [fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent] = rod_rod_rl_agent_forces.getForcesOnRods2d();
        rod_rod_rl_agent_forces.ClearCalculatedForces();
        // rods.addForces(fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent);
        for(int rod_index = num_of_vicsek_rods; rod_index < parameter.num_of_rods; rod_index++){
            rods.addForceToRod(rod_index, fx_rod_rod_rl_agent[rod_index], fy_rod_rod_rl_agent[rod_index], torque_rod_rod_rl_agent[rod_index]);
        }

        // compute rod-rod repulsive forces
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        // compute rod-obstacle repulsive forces
        for (size_t i = 0; i < rod_obstacle_forces.size(); i++) {
            rod_obstacle_forces.at(i).compute(rods);
            auto [fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle] = rod_obstacle_forces.at(i).getForcesOnRods();
            rod_obstacle_forces.at(i).clear();
            rods.addForces(fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle);
        }

        // compute velocity from total forces
        rods.calcVelocitiesFromForces();

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        // update for next step
        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        // RL: Get new state and calculate reward
        std::vector<std::vector<double>> new_state = rods.getAll1DVisionArray(r_v, vision_ray_count);
        std::vector<double> current_reward = rods.getAllComplexActiveWorkByRod(reward_accum_delta);
        std::vector<double> reward = current_reward; //calculateWeightedAverageReward(current_reward, reward_history, reward_history_length);

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show reward
            std::cout << "Current Reward is " << reward << std::endl;
        }

        // output data
        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
            rl_agent.writeTrainingLog(root_directory_of_data, phase_name, steps, total_steps_digits, reward);
        }

        // RL: insert to buffer
        // buffer insert reward and is_terminate
        std::vector<bool> is_terminal(reward.size(), false);
        if (steps == total_steps - 1) {
            std::fill(is_terminal.begin(), is_terminal.end(), true);
        }
        rl_agent.addAllReward(reward, is_terminal);

        // Assign new state as current state for next time step
        state = new_state;

        if (steps != 0){
            // RL: Agent learns from the action
            if (steps % update_freq == 0 || steps == total_steps - 1){
                rl_agent.update();
            }

            // RL: decay PPO agent action std
            if (steps % action_std_decay_freq == 0){
                rl_agent.decayActionStd(action_std_decay_rate, min_action_std);
            }

            // RL: save model
            if (steps % save_model_freq == 0 || steps == total_steps - 1){
                rl_agent.save("saved_policy.pt");
            }
        }
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}

ObstaclesInPeriodicCompression::ObstaclesInPeriodicCompression(
    Rods &rods,
    ParametersForRods &parameter_for_rods,
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles
) : PeriodicCompression(rods, parameter_for_rods),
    parameters_for_obstacles(parameters_for_obstacles)
{
    obstacles.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.emplace_back(parameters_for_obstacles.at(i), 1000);
    }

    std::cout << "-- Parameters for Rods in Compression --" << std::endl;
    this->parameter.DisplayParameters();

    std::cout << "-- Parameters for Obstacles in Compression --" << std::endl;
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        std::cout << "Obstacle " << i << ":" << std::endl;
        parameters_for_obstacles.at(i).displayParameters();
    }
}

ObstaclesInPeriodicCompression::~ObstaclesInPeriodicCompression()
{
}

void ObstaclesInPeriodicCompression::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_linear_dimension, double final_linear_dimension)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    double y_max = phase_parameter_json["y_max"].template get<double>();

    double linear_dimension = initial_linear_dimension;
    const double ratio = std::pow(final_linear_dimension / initial_linear_dimension, 1. / total_steps);

    // declare rod-rod forces
    parameter_json["time_scaled_interaction_strength_between_rod_rod"]
        = parameter_json["interaction_strength_between_rod_rod"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength_between_rod_rod"].template get<double>());

    // declare rod-obstacle forces
    parameter_json["time_scaled_interaction_strength_between_rod_obstacle"]
        = parameter_json["interaction_strength_between_rod_obstacle"].template get<double>() / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    std::vector<ParametersForForces> parameters_for_forces;
    parameters_for_forces.reserve(parameters_for_obstacles.size());
    std::vector<RodsCircularObstacleYukawaForces> rod_obstacle_forces;
    rod_obstacle_forces.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        parameters_for_forces.emplace_back(parameter_json["time_scaled_interaction_strength_between_rod_obstacle"].template get<double>(), parameter.diameter_of_segment);
        rod_obstacle_forces.emplace_back(parameter, parameters_for_obstacles.at(i), parameters_for_forces.at(i));
    }

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        // compute rod-rod forces
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        // compute rod-obstacle forces
        for (size_t i = 0; i < rod_obstacle_forces.size(); i++) {
            rod_obstacle_forces.at(i).compute(rods);
            auto [fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle] = rod_obstacle_forces.at(i).getForcesOnRods();
            rod_obstacle_forces.at(i).clear();
            rods.addForces(fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle);
        }


        // compute velocity from total forces
        rods.calcVelocitiesFromForces();

        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
            // output obstacle shape as discrete points for each step
            for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
                obstacles.at(i).writeData(root_directory_of_data, phase_name, i, steps, total_steps_digits);
            }
        }

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        // update for next step
        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        // compression of system
        linear_dimension *= ratio;
        x_max = linear_dimension;
        y_max = linear_dimension;
        // compression of rods
        rods.scalePosition(ratio);
        // compression of obstacles
        for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
            auto [center_x, center_y] = parameters_for_obstacles.at(i).getPositionOfCenter();
            obstacles.at(i).setPositionOfCenter(center_x * ratio, center_y * ratio);
            obstacles.at(i).calcPointsOnBoundary();
            rod_obstacle_forces.at(i).setCenterOfCircularObstacle(center_x * ratio, center_y * ratio);
        }
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}

ObstaclesSteepeningInPeriodic::ObstaclesSteepeningInPeriodic(
    Rods &rods,
    ParametersForRods &parameter_for_rods,
    std::vector<ParametersForCircularBoundary> &parameters_for_obstacles
) : rods(rods),
    parameter(parameter_for_rods),
    parameters_for_obstacles(parameters_for_obstacles)
{
    obstacles.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.emplace_back(parameters_for_obstacles.at(i), 1000);
    }

    std::cout << "-- Parameters for Rods in Obstacles Steepening --" << std::endl;
    this->parameter.DisplayParameters();

    std::cout << "-- Parameters for Obstacles in Obstacles Steepening --" << std::endl;
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        std::cout << "Obstacle " << i << ":" << std::endl;
        parameters_for_obstacles.at(i).displayParameters();
    }
}

ObstaclesSteepeningInPeriodic::~ObstaclesSteepeningInPeriodic()
{
}

void ObstaclesSteepeningInPeriodic::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_interaction_strength, double final_interaction_strength)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    double y_max = phase_parameter_json["y_max"].template get<double>();

    // output obstacle shape as discrete points
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        obstacles.at(i).writeData(root_directory_of_data, phase_name, i);
    }

    // declare rod-rod forces
    parameter_json["time_scaled_interaction_strength_between_rod_rod"]
        = parameter_json["interaction_strength_between_rod_rod"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength_between_rod_rod"].template get<double>());

    // declare rod-obstacle forces
    parameter_json["time_scaled_initial_interaction_strength_between_rod_obstacle"]
        = initial_interaction_strength / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    parameter_json["time_scaled_final_interaction_strength_between_rod_obstacle"]
        = final_interaction_strength / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    // assume final_strength > initial_strength
    const double ratio = std::pow(final_interaction_strength / initial_interaction_strength, 1. / total_steps);

    std::vector<ParametersForForces> parameters_for_forces;
    parameters_for_forces.reserve(parameters_for_obstacles.size());
    std::vector<RodsCircularObstacleCappedLennardJonesForces> rod_obstacle_forces;
    rod_obstacle_forces.reserve(parameters_for_obstacles.size());
    for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
        parameters_for_forces.emplace_back(parameter_json["time_scaled_initial_interaction_strength_between_rod_obstacle"].template get<double>());
        rod_obstacle_forces.emplace_back(parameter, parameters_for_obstacles.at(i), parameters_for_forces.at(i));
    }

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        // compute rod-rod forces
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        // compute rod-obstacle forces
        for (size_t i = 0; i < parameters_for_obstacles.size(); i++) {
            rod_obstacle_forces.at(i).compute(rods);
            auto [fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle] = rod_obstacle_forces.at(i).getForcesOnRods();
            rod_obstacle_forces.at(i).clear();
            rods.addForces(fx_rod_obstacle, fy_rod_obstacle, torque_rod_obstacle);
        }

        // compute velocity from total forces
        rods.calcVelocitiesFromForces();

        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
        }

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        // update for next step
        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        // steepening obstacles
        for (size_t i = 0; i < rod_obstacle_forces.size(); i++) {
            const double strength = rod_obstacle_forces.at(i).getInteractionStrength();
            rod_obstacle_forces.at(i).setInteractionStrength(strength * ratio);
        }
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}
