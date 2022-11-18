# system imports
import json
import os
import sys
# local imports
sys.path.append('../')
from utils.paths_handling import project_root_dir
from training.trainer import acromonk_training

# get all parameters
k_trains = 1
for k in range(k_trains):
    with open(os.path.join(project_root_dir, 'reinforcement_learning', 'training', 'parameters.json')) as config:
        parameters = json.load(config)

    # do the training and saving
    acromonk_training(parameters)

