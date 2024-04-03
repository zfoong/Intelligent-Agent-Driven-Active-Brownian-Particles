import argparse
import json

import make_fig_of_snapshot as mksnap

parameter_file = "data/" + "parameters_used.json"
print("parameter file is " + parameter_file)
with open(parameter_file) as f:
    params = json.load(f)
simulation_phases = params["phases"]
step_interval_for_output = params["step_interval_for_output"]

parser = argparse.ArgumentParser()
parser.add_argument(
    "--phase", "-p",
    nargs="?",
    type=int,
    default=0,
    help="phase index"
)
args = parser.parse_args()
phase_index = args.phase

simulation_phase = simulation_phases[phase_index]
total_steps = simulation_phase["total_steps"]
steps_digits = len(str(total_steps))
ranges = {
    "xmin": simulation_phase["x_min"],
    "xmax": simulation_phase["x_max"],
    "ymin": simulation_phase["y_min"],
    "ymax": simulation_phase["y_max"],
}
root_folderpath = "data/" + simulation_phase["name"] + "/"
data_folderpath = root_folderpath + "segments/"
fig_folderpath = root_folderpath + "fig/"
if with_boundary is True:
    boundary_folderpath = root_folderpath + "boundary/"

for time in range(0, total_steps, step_interval_for_output):
    time_str = str(time).zfill(steps_digits)
    data_filename = "segments" + time_str + ".dat"
    fig_filename = "segments" + time_str + ".png"

    if with_boundary is not True:
        mksnap.with_head(
            ranges, data_folderpath, data_filename, fig_folderpath, fig_filename
        )
        continue

    boundary_num = simulation_phase["boundary_num"]
    if boundary_num == 1:
        boundary_filename_list = []
        if simulation_phase["is_boundary_modified"] is True:
            boundary_filename_list = ["boundary" + time_str + ".dat"]
        else:
            boundary_filename_list = ["boundary.dat"]

    else:
        boundary_filename_list = []
        if simulation_phase["is_boundary_modified"] is True:
            for i in range(1, 1 + boundary_num):
                boundary_filename_list.append(
                    "boundary" + str(i) + "_" + time_str + ".dat"
                )
        else:
            for i in range(1, 1 + boundary_num):
                boundary_filename_list.append("boundary" + str(i) + ".dat")

    mksnap.with_head(
        ranges,
        data_folderpath,
        data_filename,
        fig_folderpath,
        fig_filename,
        boundary_folderpath,
        boundary_filename_list,
    )
