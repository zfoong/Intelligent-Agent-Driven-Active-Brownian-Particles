import json
import os


def mkdir(working_dir, with_boundary=True):
    data_dir = working_dir + "data/"
    print("data directory is " + data_dir)

    parameter_file = working_dir + "parameters.json"
    print("parameter file is " + parameter_file)
    with open(parameter_file) as f:
        params = json.load(f)
    simulation_phases = params["phases"]

    if with_boundary is True:
        data_types = ["rods", "segments", "boundary", "fig"]
    else:
        data_types = ["rods", "segments", "fig"]

    for simulation_phase in simulation_phases:
        for data_type in data_types:
            os.makedirs(
                data_dir + simulation_phase["name"] + "/" + data_type, exist_ok=True
            )


if __name__ == "__main__":
    # config root directory
    working_dir = os.getcwd() + "/"
    print("working directory is " + working_dir)

    mkdir(working_dir)
