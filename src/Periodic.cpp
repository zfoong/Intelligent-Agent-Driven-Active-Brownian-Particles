#include <iostream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <vector>
#include <deque>
#include <numeric>

#include "Periodic.hpp"
#include "Rods.hpp"
#include "ParametersForRods.hpp"
#include "ForcesOnSegments2d.hpp"
#include "util.hpp"
#include "RLAgent.hpp"

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
    int transient_phase = parameter_json["transient_phase"].template get<int>();                     // perform backprop in every n time step
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
    double vision_ray_count = 1; //16;
    double r_v = 10;

    const int num_of_intelligent_rods = static_cast<int>(parameter.num_of_rods * intelligent_rods_ratio);
    const int num_of_vicsek_rods = parameter.num_of_rods - num_of_intelligent_rods;

    parameter_json["time_scaled_interaction_strength"]
    = parameter_json["interaction_strength"].template get<double>() / parameter.num_of_segments / parameter.num_of_segments
    * time_scale * time_scale; // interaction potential coefficient scaled by time interval per time step in simulation
    
    // Initialize RL agent
    RLAgent rl_agent(0.1, 0.2, 0.1, 0, 0.8);

    std::cout << "start " << phase_name << " phase..." << std::endl;

    int total_episode = 10;
    int bufferSize = 10000; 
    const int reward_history_length = 10;
    std::vector<double> reward;

    for(int current_episode=1; current_episode < total_episode; current_episode++){

        ForcesOnSegments2d rod_rod_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
        // ForcesOnSegments2d rod_rod_vicsek_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
        ForcesOnSegments2d rod_rod_rl_agent_forces(parameter, parameter_json["time_scaled_interaction_strength"].template get<double>());
        std::vector<double> full_state = rods.getAllAngleDifferenceWithinRange(r_v);
        
        std::deque<std::vector<double>> reward_history;

        for (int steps = 0; steps < transient_phase; steps++){
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

            std::vector<double> actions(full_state.size(), 0.0);

            // RL: Apply artificial force to the intelligent active rods
            rod_rod_rl_agent_forces.calcRLAgentArtificialForcesWithPeriodic(rods, actions, x_min, x_max, y_min, y_max);
            auto [fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent] = rod_rod_rl_agent_forces.getForcesOnRods2d();
            rod_rod_rl_agent_forces.ClearCalculatedForces();
            // rods.addForces(fx_rod_rod_rl_agent, fy_rod_rod_rl_agent, torque_rod_rod_rl_agent);
            for(int rod_index = num_of_vicsek_rods; rod_index < parameter.num_of_rods; rod_index++){
                rods.addForceToRod(rod_index, fx_rod_rod_rl_agent[rod_index], fy_rod_rod_rl_agent[rod_index], torque_rod_rod_rl_agent[rod_index]);
            }

            rods.calcVelocitiesFromForces();
            rods.updateRods();
            rods.periodic(x_min, x_max, y_min, y_max);
        }

        full_state = rods.getAllAngleDifferenceWithinRange(r_v);

        std::vector<std::vector<double>> currentStateList(num_of_intelligent_rods);
        std::vector<std::vector<int>> actionIDList(num_of_intelligent_rods);
        std::vector<std::vector<double>> rewardList(num_of_intelligent_rods);
        std::vector<std::vector<double>> newStateList(num_of_intelligent_rods);

        // Main loop
        for (int steps = 0; steps < total_steps; steps++) {


            std::vector<int> actionID(num_of_intelligent_rods);
            std::vector<double> actions = rl_agent.ReturnAction(full_state, actionID);


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
            rods.updateRods();
            rods.periodic(x_min, x_max, y_min, y_max);

            // gifファイル作成のためのデータ出力終了
            if ((total_steps >= 10) && (steps % (total_steps / 10) == 0)) {  // show progress
                DisplayProgress(steps, total_steps);
            }

            // RL: Get new state and calculate reward
            // std::vector<std::vector<double>> new_state = rods.getAll1DVisionArray(r_v, vision_ray_count);
            std::vector<double> new_state = rods.getAllAngleDifferenceWithinRange(r_v);

            std::vector<double> current_reward = rods.getAllActiveWorkByRod(reward_accum_delta);
            reward = current_reward; 

            // gifファイル作成のためのデータ出力開始
            if ((steps % parameter_json["step_interval_for_output"].template get<int>()) == 0) {
                rods.writeData(root_directory_of_data, phase_name, steps, total_steps_digits);
                //rl_agent.writeTrainingLog(root_directory_of_data, phase_name, steps, total_steps_digits, reward);
            }

            // RL: insert to buffer
            // buffer insert reward and is_terminate
            std::vector<bool> is_terminal(reward.size(), false);
            if (steps == total_steps - 1) {
                std::fill(is_terminal.begin(), is_terminal.end(), true);
            }

            for (int i = 0; i < num_of_intelligent_rods; i++) {
                currentStateList[i].push_back(full_state[i]);
                actionIDList[i].push_back(actionID[i]);
                rewardList[i].push_back(reward[i]);
                newStateList[i].push_back(new_state[i]);
            }

            // Assign new state as current state for next time step
            full_state = new_state;

            if (currentStateList[0].size() >= bufferSize) {
                for (int i = 0; i < currentStateList.size(); i++) {
                    std::vector<double> _currentState = currentStateList[i];
                    std::vector<int> _actionID = actionIDList[i];
                    std::vector<double> _reward = rewardList[i];
                    std::vector<double> _newState = newStateList[i];
                    rl_agent.UpdateSVTable(_currentState, _actionID, _reward, _newState);
                }
                rl_agent.SortStateValueList();
                rl_agent.UpdateTPMatrix();
                for (int i = 0; i < currentStateList.size(); i++) {
                    currentStateList[i].clear();
                    actionIDList[i].clear();
                    rewardList[i].clear();
                    newStateList[i].clear();
                }
            }

            if (steps != 0){
                // RL: save model
                if (steps % save_model_freq == 0 || steps == total_steps - 1){
                    rl_agent.SaveDTable((root_directory_of_data + "/" + "DTable").c_str());
                    rl_agent.SaveSVTable((root_directory_of_data + "/" + "SVTable").c_str());
                    rl_agent.SaveTPMatrix((root_directory_of_data + "/" + "TPMatrix").c_str());
                }
            }
        }
        
        rl_agent.UpdateEpsilonDecay(current_episode, total_episode);
        rl_agent.UpdateLearningRateDecay(current_episode, total_episode);
        double epsilon = rl_agent.returnEpsilon();
        double learning_rate = rl_agent.returnLearningRate();
        std::cout << "End of episode " << current_episode << std::endl;
        std::cout << "Current Reward is " << reward << std::endl;
        std::cout << "Current epsilon is " << epsilon << std::endl;
        std::cout << "Current learning_rate is " << learning_rate << std::endl;

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
