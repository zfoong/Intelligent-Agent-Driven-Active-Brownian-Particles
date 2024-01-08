#include <vector>
#include <iostream>
#include <cmath>
#include <torch/torch.h>
#include "util.hpp"

// The following code took reference from https://github.com/nikhilbarhate99/PPO-PyTorch/tree/master

class AgentBuffer {
public:
    void clear() {
        actions.clear();
        states.clear();
        logprobs.clear();
        rewards.clear();
        state_values.clear();
        is_terminals.clear();
    }

    void addAction(double action) {
        actions.push_back(action);
    }

    void addState(std::vector<double> state) {
        states.push_back(state);
    }

    void addLogprob(double logprob) {
        logprobs.push_back(logprob);
    }

    void addReward(double reward) {
        rewards.push_back(reward);
    }

    void addStateValue(double state_value) {
        state_values.push_back(state_value);
    }

    void addIsTerminal(bool is_terminal) {
        is_terminals.push_back(is_terminal);
    }


    const std::vector<double>& getActions() const {
        return actions;
    }

    const std::vector<std::vector<double>>& getStates() const {
        return states;
    }

    const std::vector<double>& getLogprobs() const {
        return logprobs;
    }

    const std::vector<double>& getRewards() const {
        return rewards;
    }

    const std::vector<double>& getStateValues() const {
        return state_values;
    }

    const std::vector<bool>& getIsTerminals() const {
        return is_terminals;
    }


private:
    std::vector<double> actions;
    std::vector<std::vector<double>> states;
    std::vector<double> logprobs;
    std::vector<double> rewards;
    std::vector<double> state_values;
    std::vector<bool> is_terminals;
};

class RolloutBuffer {
public:
    RolloutBuffer() {}

    void clear() {
        for (auto& agent_buffer : agent_buffers) {
            agent_buffer.clear();
        }
    }

    size_t size() const {
        return agent_buffers.size();
    }

    AgentBuffer& agent(int agent_id) {
        if (agent_id >= static_cast<int>(agent_buffers.size())) {
            agent_buffers.resize(agent_id + 1);
        }
        return agent_buffers[agent_id];
    }

private:
    std::vector<AgentBuffer> agent_buffers;
};

class ActorCritic : public torch::nn::Module {
public:
    ActorCritic(int state_dim, int action_dim, double action_std_init)
    : action_dim(action_dim) {
        
        action_var = torch::full({action_dim}, action_std_init * action_std_init).to(torch::kCUDA); // Assuming using CUDA

        // Actor
        actor = torch::nn::Sequential(
                    torch::nn::Linear(state_dim, 64),
                    torch::nn::Tanh(),
                    torch::nn::Linear(64, 64),
                    torch::nn::Tanh(),
                    torch::nn::Linear(64, action_dim),
                    torch::nn::Tanh()
                );

        // Critic
        critic = torch::nn::Sequential(
                    torch::nn::Linear(state_dim, 64),
                    torch::nn::Tanh(),
                    torch::nn::Linear(64, 64),
                    torch::nn::Tanh(),
                    torch::nn::Linear(64, 1)
                );

        // Convert actor, critic, and action_var to double
        actor->to(torch::kDouble);
        critic->to(torch::kDouble);
        action_var = action_var.to(torch::kDouble);

        // Check if CUDA is available
        if (torch::cuda::is_available()) {
            // Move models to GPU if CUDA is available
            actor->to(torch::kCUDA);
            critic->to(torch::kCUDA);
            action_var = action_var.to(torch::kCUDA);
        } else {
            // Handle the case where CUDA is not available
            std::cout << "CUDA is not available. Models will run on CPU." << std::endl;
        }

        // Register modules
        register_module("actor", actor);
        register_module("critic", critic);
    }

    torch::nn::Sequential& get_actor() {
        return actor;
    }

    torch::nn::Sequential& get_critic() {
        return critic;
    }

    void setActionStd(double new_action_std) 
    {
        action_var = torch::full({action_dim}, new_action_std * new_action_std).to(torch::kCUDA);
        action_var = action_var.to(torch::kDouble);
    }

    torch::Tensor forward() 
    {
        throw std::logic_error("Forward method not implemented.");
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> act(torch::Tensor state) 
    {
        torch::Tensor action_mean = actor->forward(state);
        torch::Tensor std_dev = torch::sqrt(action_var); 
        torch::Tensor action = at::normal(action_mean, std_dev);
        // Implementing log_prob manually
        // The formula for the log probability of a normal distribution is:
        // log_prob = -((action - action_mean) ^ 2) / (2 * action_var) - log(std_dev * sqrt(2 * M_PI))
        torch::Tensor action_logprob = -torch::pow(action - action_mean, 2) / (2 * action_var) - torch::log(std_dev * std::sqrt(2 * M_PI));

        torch::Tensor state_val = critic->forward(state);

        return std::make_tuple(action, action_logprob, state_val);
    }

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> evaluate(torch::Tensor state, torch::Tensor action) {
        torch::Tensor action_mean = actor->forward(state);
        torch::Tensor std_dev = torch::sqrt(action_var); 
        torch::Tensor action_var_expanded = action_var.expand_as(action_mean);
        if (action_dim == 1) {
            action = action.reshape({-1, action_dim});
        }
        // Calculate log probabilities manually
        // Log probability for a normal distribution:
        // log_prob = -((action - action_mean) ^ 2) / (2 * action_var) - log(std_dev * sqrt(2 * M_PI))
        torch::Tensor action_logprobs = -torch::pow(action - action_mean, 2) / (2 * action_var_expanded) - torch::log(std_dev * std::sqrt(2 * M_PI));
        // Calculate entropy manually
        // Entropy for a normal distribution: 0.5 + 0.5 * log(2 * M_PI * action_var)
        torch::Tensor dist_entropy = 0.5 + 0.5 * torch::log(2 * M_PI * action_var_expanded);
        torch::Tensor state_values = critic->forward(state);

        return std::make_tuple(action_logprobs, state_values, dist_entropy);
    }

private:
    torch::nn::Sequential actor, critic;
    int action_dim;
    torch::Tensor action_var;
};

class PPO {
public:
    PPO(int state_dim, int action_dim, double lr_actor, double lr_critic, double gamma, int K_epochs, double eps_clip, double action_std_init = 0.6)
    : gamma(gamma), 
      eps_clip(eps_clip), 
      K_epochs(K_epochs), 
      buffer() 
    {
        action_std = action_std_init;

        policy = std::make_shared<ActorCritic>(state_dim, action_dim, action_std_init);
        policy->to(torch::kCUDA); // Assuming using CUDA

        policy_old = std::make_shared<ActorCritic>(state_dim, action_dim, action_std_init);
        policy_old->to(torch::kCUDA); // Assuming using CUDA
        torch::save(policy, "policy_old.pt"); // Save initial state
        torch::load(policy_old, "policy_old.pt"); // Load into policy_old

        // Create AdamOptions for actor and critic
        auto actor_options = std::make_unique<torch::optim::AdamOptions>(lr_actor);
        auto critic_options = std::make_unique<torch::optim::AdamOptions>(lr_critic);

        // Create parameter groups for actor and critic
        std::vector<torch::optim::OptimizerParamGroup> param_groups;
        param_groups.emplace_back(policy->get_actor()->parameters(), std::move(actor_options));
        param_groups.emplace_back(policy->get_critic()->parameters(), std::move(critic_options));

        // Create the Adam optimizer with these parameter groups
        optimizer = std::make_unique<torch::optim::Adam>(param_groups);

        //actor_optimizer = std::make_unique<torch::optim::Adam>(policy->get_actor()->parameters(), torch::optim::AdamOptions(lr_actor));
        //critic_optimizer = std::make_unique<torch::optim::Adam>(policy->get_critic()->parameters(), torch::optim::AdamOptions(lr_critic));

        MseLoss = torch::nn::MSELoss();
    }

    void setActionStd(double new_action_std) 
    {
        action_std = new_action_std;
        policy->setActionStd(new_action_std);
        policy_old->setActionStd(new_action_std);
    }

    void decayActionStd(double action_std_decay_rate, double min_action_std) 
    {
        action_std -= action_std_decay_rate;
        action_std = std::max(action_std, min_action_std);
        std::cout << "setting actor output action std to: " << action_std << std::endl;
        setActionStd(action_std);
    }

    double selectAction(int agent_id, std::vector<std::vector<double>> state) 
    {
        torch::Tensor state_tensor = vector2DToTensor(state).to(torch::kCUDA);

        // Get action, log probability, and state value from the policy
        auto [action, action_logprob, state_val] = policy_old->act(state_tensor);

        std::vector<double> state_vector = tensorToVector(state_tensor);

        // Storing experience into agent buffer
        auto& agent_buffer = buffer.agent(agent_id);
        agent_buffer.addState(state_vector);
        agent_buffer.addAction(action.item<double>());
        agent_buffer.addLogprob(action_logprob.item<double>());
        agent_buffer.addStateValue(state_val.item<double>());

        double action_value = action.item<double>();
        return action_value;
    }

    void addReward(int agent_id, double reward, bool is_terminal)
    {
        auto& agent_buffer = buffer.agent(agent_id);
        agent_buffer.addReward(reward);
        agent_buffer.addIsTerminal(is_terminal);
    }

    std::vector<torch::Tensor> convertToTensors(const std::vector<double>& vec) {
        std::vector<torch::Tensor> tensors;
        for (double value : vec) {
            tensors.push_back(torch::tensor(value));
        }
        return tensors;
    }

    std::vector<torch::Tensor> convertToTensors(const std::vector<std::vector<double>>& vec) {
        std::vector<torch::Tensor> tensors;

        for (const auto& inner_vec : vec) {
            tensors.push_back(torch::tensor(inner_vec));
        }

        return tensors;
    }

    // TODO make sure this is not slowing down the update
    std::vector<double> tensorToVector(const torch::Tensor& tensor) {
        // Ensure the tensor is on CPU for extraction
        torch::Tensor cpu_tensor = tensor.to(torch::kCPU);
        std::vector<double> vector(cpu_tensor.data_ptr<double>(), cpu_tensor.data_ptr<double>() + cpu_tensor.numel());

        return vector;
    }

    void update() 
    {
        int buffer_size = static_cast<int>(buffer.size());
        for (int agent_id = 0; agent_id < buffer_size; agent_id++) {
            auto& agent_buffer = buffer.agent(agent_id);

            // Monte Carlo estimate of returns for each agent
            std::vector<double> rewards;
            double discounted_reward = 0.0;
            for (int i = agent_buffer.getRewards().size() - 1; i >= 0; i--) {
                if (agent_buffer.getIsTerminals()[i]) {
                    discounted_reward = 0;
                }
                discounted_reward = agent_buffer.getRewards()[i] + (gamma * discounted_reward);
                rewards.insert(rewards.begin(), discounted_reward);
            }

            // Normalizing the rewards
            torch::Tensor rewards_tensor = torch::tensor(rewards).to(torch::kCUDA).to(torch::kDouble); // Assuming CUDA
            rewards_tensor = (rewards_tensor - rewards_tensor.mean()) / (rewards_tensor.std() + 1e-7);

            auto statesTensors = convertToTensors(agent_buffer.getStates());
            torch::Tensor old_states = torch::stack(statesTensors).detach().to(torch::kCUDA);

            auto actionsTensors = convertToTensors(agent_buffer.getActions());
            torch::Tensor old_actions = torch::stack(actionsTensors).squeeze().detach().to(torch::kCUDA);

            auto logprobsTensors = convertToTensors(agent_buffer.getLogprobs());
            torch::Tensor old_logprobs = torch::stack(logprobsTensors).squeeze().detach().to(torch::kCUDA);

            auto stateValuesTensors = convertToTensors(agent_buffer.getStateValues());
            torch::Tensor old_state_values = torch::stack(stateValuesTensors).squeeze().detach().to(torch::kCUDA);

            old_states = old_states.to(torch::kDouble);
            old_actions = old_actions.view({-1, 1}).to(torch::kDouble);
            old_logprobs = old_logprobs.view({-1, 1}).to(torch::kDouble);
            old_state_values = old_state_values.view({-1, 1}).to(torch::kDouble);

            // Calculate advantages
            torch::Tensor advantages = rewards_tensor.detach() - old_state_values.detach();

            // Optimize policy for K epochs
            for (int i = 0; i < K_epochs; i++) {
                // Evaluating old actions and values

                auto [logprobs, state_values, dist_entropy] = policy->evaluate(old_states, old_actions);
                state_values = state_values.squeeze();

                // Finding the ratio (pi_theta / pi_theta_old)
                torch::Tensor ratios = torch::exp(logprobs - old_logprobs.detach());

                // Finding Surrogate Loss
                torch::Tensor surr1 = ratios * advantages;
                torch::Tensor surr2 = torch::clamp(ratios, 1 - eps_clip, 1 + eps_clip) * advantages;
                torch::Tensor loss = -torch::min(surr1, surr2) + 0.5 * MseLoss(state_values, rewards_tensor) - 0.01 * dist_entropy;

                optimizer->zero_grad();
                loss.mean().backward();
                optimizer->step();

                //torch::Tensor actor_loss = -torch::min(surr1, surr2) - 0.01 * dist_entropy;

                // Take gradient step for actor
                //actor_optimizer->zero_grad();
                //actor_loss.mean().backward();
                //actor_optimizer->step();

                // Loss for critic
                //torch::Tensor critic_loss = 0.5 * MseLoss(state_values, rewards_tensor);

                // Take gradient step for critic
                //critic_optimizer->zero_grad();
                //critic_loss.mean().backward();
                //critic_optimizer->step();
            }
            agent_buffer.clear();
        }

        // Copy new weights into old policy
        torch::save(policy, "policy.pt");
        torch::load(policy_old, "policy.pt");

        buffer.clear();
    }

    void save(std::string checkpoint_path) 
    {
        torch::save(policy_old, checkpoint_path);
    }

    void load(std::string checkpoint_path) 
    {
        torch::load(policy_old, checkpoint_path);
        torch::load(policy, checkpoint_path);
    }

private:
    double gamma;
    double eps_clip;
    int K_epochs;
    RolloutBuffer buffer;
    std::shared_ptr<ActorCritic> policy;
    std::shared_ptr<ActorCritic> policy_old;
    std::unique_ptr<torch::optim::Adam> actor_optimizer;  // Change to unique_ptr
    std::unique_ptr<torch::optim::Adam> critic_optimizer; // Change to unique_ptr
    std::unique_ptr<torch::optim::Adam> optimizer;  // Change to unique_ptr
    torch::nn::MSELoss MseLoss;
    double action_std;
};


class RLAgent {
public:
    RLAgent(int state_dim, int action_dim, double lr_actor, double lr_critic, double gamma, int K_epochs, double eps_clip, double action_std_init = 0.6)
        : ppo(state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip, action_std_init) 
    {
        // Additional initialization if needed
    }

    void decayActionStd(double action_std_decay_rate = 0.05, double min_action_std = 0.1) 
    {
        ppo.decayActionStd(action_std_decay_rate, min_action_std);
    }

    std::vector<double> selectAllAction(std::vector<std::vector<double>> state) 
    {
        std::vector<double> all_actions;

        for (int agent_id = 0; agent_id < static_cast<int>(state.size()); agent_id++) 
        {
            std::vector<std::vector<double>> agent_state = {state[agent_id]};
            double action = ppo.selectAction(agent_id, agent_state);
            all_actions.push_back(action);
        }
        return all_actions;
    }

    void addAllReward(std::vector<double> reward, std::vector<bool> is_terminal)
    {
        for (int agent_id = 0; agent_id < static_cast<int>(reward.size()); agent_id++) 
        {
            ppo.addReward(agent_id, reward[agent_id], is_terminal[agent_id]);
        }
    }

    void update()
    {
        ppo.update();
    }

    void save(std::string checkpoint_path)
    {
        ppo.save(checkpoint_path);
    }

    void load(std::string checkpoint_path)
    {
        ppo.load(checkpoint_path);
    }

private:
    PPO ppo; 
};