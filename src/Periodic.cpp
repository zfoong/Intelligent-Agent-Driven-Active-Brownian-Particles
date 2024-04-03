#include <iostream>
#include <nlohmann/json.hpp>
#include <cmath>

#include "Periodic.hpp"
#include "Rods.hpp"
#include "ParametersForRods.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"
#include "RLAgent.cpp"

using json = nlohmann::json;

Periodic::Periodic(ActiveRods &active_rods, ParametersForActiveRods &parameter_for_active_rods) : rods(active_rods), parameter(parameter_for_active_rods)
{
    std::cout << "-- Parameters for Active Rods in Simulation --" << std::endl;
    this->parameter.DisplayParameters();
}

Periodic::~Periodic()
{
}

PeriodicCompression::PeriodicCompression(Rods &rods, ParametersForRods &parameter_for_rods) : rods(rods), parameter(parameter_for_rods)
{
    std::cout << "-- Parameters for Rods in Compression --" << std::endl;
    this->parameter.DisplayParameters();
}

PeriodicCompression::~PeriodicCompression()
{
}

void Periodic::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale)
{
    json phase_parameter_json = parameter_json["phases"][phase_index];

    const std::string phase_name = phase_parameter_json["name"].template get<std::string>();
    const int total_steps = phase_parameter_json["total_steps"].template get<int>();
    const int total_steps_digits = std::to_string(total_steps).length();
    const double x_min = phase_parameter_json["x_min"].template get<double>();
    const double x_max = phase_parameter_json["x_max"].template get<double>();
    const double y_min = phase_parameter_json["y_min"].template get<double>();
    const double y_max = phase_parameter_json["y_max"].template get<double>();

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
    double vision_ray_count = 8;
    double r_v = 5;

    const int num_of_intelligent_rods = static_cast<int>(parameter.num_of_rods * intelligent_rods_ratio);
    const int num_of_vicsek_rods = parameter.num_of_rods - num_of_intelligent_rods;

    parameter_json["time_scaled_interaction_strength"]
    = parameter_json["interaction_strength"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
    * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
    // ForcesOnSegments2d rod_rod_vicsek_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
    ForcesOnSegments2d rod_rod_rl_agent_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());

    
    // Initialize RL agent
    RLAgent rl_agent(vision_ray_count, 1, lr_actor, lr_critic, gamma, K_epochs, eps_clip, action_std);

    std::cout << "start " << phase_name << " phase..." << std::endl;

    // RL: Get current state of the environment
    std::vector<std::vector<double>> state = rods.getAll1DVisionArray(r_v, vision_ray_count);

    // Main loop
    for (int steps = 0; steps < total_steps; steps++) {

        // RL: Agent decides action based on current state
        std::vector<double> actions = rl_agent.selectAllAction(state);

        // Applying forces to rods
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        // rod_rod_vicsek_forces.calcRodRodVicsekForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        // auto [fx_rod_rod_vicsek, fy_rod_rod_vicsek, torque_rod_rod_vicsek] = rod_rod_vicsek_forces.getForcesOnRods2d();
        // rod_rod_vicsek_forces.ClearCalculatedForces();
        // rods.addForces(fx_rod_rod_vicsek, fy_rod_rod_vicsek, torque_rod_rod_vicsek);
        // for(int rod_index = 0; rod_index < num_of_intelligent_rods; rod_index++){
        //    rods.addForceToRod(rod_index, fx_rod_rod_vicsek[rod_index], fy_rod_rod_vicsek[rod_index], torque_rod_rod_vicsek[rod_index]);
        // }

        // RL: Apply artificial force to the intelligent active rods
        rod_rod_rl_agent_forces.calcRLAgentArtificialForcesWithPeriodic(rods, actions, x_min, x_max, y_min, y_max);
        auto [fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent] = rod_rod_rl_agent_forces.getForcesOnRods2d();
        rod_rod_rl_agent_forces.ClearCalculatedForces();
        // rods.addForces(fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent);
        for(int rod_index = num_of_vicsek_rods; rod_index < parameter.num_of_rods; rod_index++){
            rods.addForceToRod(rod_index, fx_rod_rod_rl_agent[rod_index], fy_rod_rod_rl_agent[rod_index], torque_rod_rod_rl_agent[rod_index]);
        }

        rods.calcVelocitiesFromForces();

        // gifファイル作成のためのデータ出力開始
        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
        }
        // gifファイル作成のためのデータ出力終了
        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        // RL: Get new state and calculate reward
        std::vector<std::vector<double>> new_state = rods.getAll1DVisionArray(r_v, vision_ray_count);
        std::vector<double> reward = rods.getAllActiveWorkByRodWithinRange(reward_accum_delta, range_neighbor);

        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show reward
            std::cout << "Current Reward is " << reward << std::endl;
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

void PeriodicCompression::Run(json parameter_json, int phase_index, std::string root_directory_of_data, double time_scale, double initial_linear_dimension, double final_linear_dimension)
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

    parameter_json["time_scaled_interaction_strength"]
        = parameter_json["interaction_strength"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
        * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());

    std::cout << "start " << phase_name << " phase..." << std::endl;
    for (int steps = 0; steps < total_steps; steps++) {
        rod_rod_forces.calcRodRodYukawaForcesWithPeriodic(rods, x_min, x_max, y_min, y_max);
        rod_rod_forces.calcForcesOnRods2d(rods);
        auto [fx_rod_rod, fy_rod_rod, torque_rod_rod] = rod_rod_forces.getForcesOnRods2d();
        rod_rod_forces.ClearCalculatedForces();
        rods.addForces(fx_rod_rod, fy_rod_rod, torque_rod_rod);

        rods.calcVelocitiesFromForces();

        // gifファイル作成のためのデータ出力開始
        if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
            rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
        }
        // gifファイル作成のためのデータ出力終了
        if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
            DisplayProgress(steps, total_steps);
        }

        rods.updateRods();
        rods.periodic(x_min, x_max, y_min, y_max);

        linear_dimension *= ratio;
        x_max = linear_dimension;
        y_max = linear_dimension;
        rods.scalePosition(ratio);
    }
    std::cout << "finish " << phase_name << " phase!" << std::endl;
}
