from stable_baselines3 import SAC, PPO, DDPG
import os
import json
from environment.acromonk import AcroMonkEnv
from utils.make_mujoco_simulator import make_mujoco_simulator


def make_rl_model(env, rl_parameters):
    if rl_parameters['algorithm'] == 'SAC':
        model = SAC('MlpPolicy', env, verbose=1, device='cpu', use_sde=True)
    elif rl_parameters['algorithm'] == 'PPO':
        model = PPO('MlpPolicy', env, verbose=1, device='cpu', use_sde=False)
    elif rl_parameters['algorithm'] == 'DDPG':
        model = DDPG('MlpPolicy', env, verbose=1, device='cpu')
    else:
        raise NotImplementedError(f'Algorithm choice {rl_parameters["algorithm"]} not recognized or implemented yet :<')

    return model


def load_rl_model(model_data_folder, checkpoint_to_load):
    with open(os.path.join(model_data_folder, 'parameters', 'parameters.json')) as param_file:
        parameters = json.load(param_file)

    algorithm = parameters['rl_model']['algorithm']
    if algorithm == 'PPO':
        model_class = PPO
    elif algorithm == 'SAC':
        model_class = SAC
    elif algorithm == 'DDPG':
        model_class = DDPG
    else:
        raise NotImplementedError(f'Models of type {algorithm} not implemented yet')

    if checkpoint_to_load == 'final':
        model = model_class.load(os.path.join(model_data_folder, 'model_final.zip'),
                                 device='cpu')
    elif checkpoint_to_load == 'best':
        model = model_class.load(os.path.join(model_data_folder, 'best_model.zip'),
                                 device='cpu')
    else:
        model = model_class.load(os.path.join(model_data_folder, f'model_{checkpoint_to_load}_steps.zip'),
                                 device='cpu')

    simulator = make_mujoco_simulator(parameters['simulation'])
    env = AcroMonkEnv(simulator, parameters['environment'], parameters['reward_setup'])

    return model, parameters, env
