import datetime
import json
import os
import subprocess

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
    # stdout of strlist_exec_doublet does not output in console
    # try:
    #     proc = subprocess.run(strlist_exec_doublet, capture_output=True)
    # except subprocess.CalledProcessError as e:
    #     print(e.output)
    # print(proc.stdout.decode("utf-8"))


if __name__ == "__main__":
    print(datetime.datetime.today())

    # config root directory
    working_dir = os.getcwd() + "/"
    print("working directory is " + working_dir)

    parameter_file = working_dir + "parameters.json"
    with open(parameter_file) as f:
        params = json.load(f)
    project_name = params["project"]

    exec_file = working_dir + "bin/" + project_name
    print("exec file is " + exec_file)

    mkdir(working_dir)

    mkdir_data(working_dir, parameter_file)

    run(exec_file, parameter_file)

    print(datetime.datetime.today())
