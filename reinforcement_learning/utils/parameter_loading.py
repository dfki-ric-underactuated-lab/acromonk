# util for loading parameters
import os
import json


def load_parameters(path):

    with open(os.path.join(path, 'parameters', 'simulation_parameters.json')) as sim_config:
        sim_parameters = json.load(sim_config)
    # rl parameters
    with open(os.path.join(path, 'parameters', 'rl_parameters.json')) as rl_config:
        rl_parameters = json.load(rl_config)

    return sim_parameters, rl_parameters
