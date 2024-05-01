import datetime
import json
import os
import subprocess
import shutil

from setup import mkdir

def mkdir_data(working_dir, parameter_file):
    data_dir = working_dir + "data/"
    print("data directory is " + data_dir)

    print("parameter file is " + parameter_file)
    with open(parameter_file) as f:
        params = json.load(f)
    simulation_phases = params["phases"]
    data_types = params["data_types"]

    for simulation_phase in simulation_phases:
        for data_type in data_types:
            os.makedirs(
                data_dir + params["name"] + "/" + simulation_phase["name"] + "/" + data_type,
                exist_ok=True
            )

def run(exec_file, parameter_file):
    subprocess.run(["chmod", "+x", exec_file])
    strlist_exec_doublet = [exec_file, parameter_file]
    print(strlist_exec_doublet)
    proc = subprocess.Popen(strlist_exec_doublet, shell=False)
    proc.communicate()

def process_experiments(pending_dir, completed_dir, exec_file, working_dir):
    # List all parameter files in the pending directory
    parameter_files = [f for f in os.listdir(pending_dir) if f.endswith('.json')]
    for parameter_file in parameter_files:
        full_path = os.path.join(pending_dir, parameter_file)
        print(f"Running experiment with: {full_path}")
        
        mkdir_data(working_dir, full_path)

        # Run the experiment
        run(exec_file, full_path)
        
        # Move the completed parameter file to the completed directory
        shutil.move(full_path, os.path.join(completed_dir, parameter_file))
        print(f"Moved {parameter_file} to {completed_dir}")

if __name__ == "__main__":
    print(datetime.datetime.today())

    working_dir = os.getcwd() + "/"
    print("working directory is " + working_dir)
    
    pending_dir = os.path.join(working_dir, "execute/experiments/pending_experiments")
    completed_dir = os.path.join(working_dir, "execute/experiments/completed_experiments")
    mkdir(pending_dir)  
    mkdir(completed_dir) 
    
    parameter_file = working_dir + "parameters.json"
    with open(parameter_file) as f:
        params = json.load(f)
    project_name = params["project"]
    
    exec_file = os.path.join(working_dir, "bin", project_name)
    print("exec file is " + exec_file)

    process_experiments(pending_dir, completed_dir, exec_file, working_dir)

    print(datetime.datetime.today())
