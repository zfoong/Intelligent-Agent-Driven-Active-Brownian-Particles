import json
import os
import datetime
import argparse

def create_parameter_files(param_name, start, end, num_experiments, name):
    working_dir = os.getcwd() + "/"
    parameter_file = working_dir + "parameters.json"
    
    with open(parameter_file, 'r') as f:
        base_params = json.load(f)

    interval = (end - start) / (num_experiments - 1) 

    file_suffix = name
    if file_suffix == '':
        file_suffix = base_params['name']

    current_value = start
    while current_value <= end:
        base_params[param_name] = current_value
        base_params['name'] = f"{file_suffix}_{param_name}_{current_value}"  # Use base name or provided suffix to create new name for the experiment file
        new_file_name = f"{working_dir}execute/experiments/pending_experiments/{base_params['name']}.json"
        
        with open(new_file_name, 'w') as f:
            json.dump(base_params, f, indent=4)
        
        print(f"File saved: {new_file_name}")
        
        current_value += interval
        if current_value > end and current_value - interval < end:
            current_value = end  # Ensure the last value is used even if interval skips it

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create parameter files for experiments.')
    parser.add_argument('param_name', type=str, help='The parameter name to vary.')
    parser.add_argument('start', type=float, help='The start value for the parameter.')
    parser.add_argument('end', type=float, help='The end value for the parameter.')
    parser.add_argument('num_experiments', type=int, help='The number of experiments/files to create.')
    parser.add_argument('name', type=str, default='', help='The suffix of the experiments')

    args = parser.parse_args()

    print(datetime.datetime.today())
    working_dir = os.getcwd() + "/"
    print("working directory is " + working_dir)

    create_parameter_files(args.param_name, args.start, args.end, args.num_experiments, args.name)
