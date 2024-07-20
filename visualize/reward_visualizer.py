import argparse
import json
import os
import matplotlib.pyplot as plt

parameter_file = "data/B/" + "parameters_used.json"
print("parameter file is " + parameter_file)
with open(parameter_file) as f:
    params = json.load(f)
simulation_phases = params["phases"]

parser = argparse.ArgumentParser()
parser.add_argument(
    "--phase", "-p",
    nargs="?",
    type=int,
    default=1,
    help="phase index"
)
args = parser.parse_args()
phase_index = args.phase

simulation_phase = simulation_phases[phase_index]
root_folderpath = "data/" + params["name"] + "/" + simulation_phase["name"] + "/"
data_folderpath = root_folderpath + "training_logs/"

def extract_reward(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('Average Reward:'):
                return float(line.split(':')[1].strip())

def plot_rewards(folder_path, root_folderpath):
    rewards = []
    file_indices = []

    # Collect all data files
    files = sorted([f for f in os.listdir(folder_path) if f.endswith('.dat')])
    
    # Extract rewards
    for file in files:
        index = int(file.split('_')[2].split('.')[0])
        reward = extract_reward(os.path.join(folder_path, file))
        file_indices.append(index)
        rewards.append(reward)
    
    # Plotting
    plt.figure(figsize=(6, 4), dpi=300)
    plt.plot(file_indices, rewards)
    plt.title('Average Rewards Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Average Reward')
    plt.grid(True)
    #plt.show()
    plt.savefig(root_folderpath + 'average_rewards_plot.png')
    plt.close()

# Example usage
plot_rewards(data_folderpath, root_folderpath)