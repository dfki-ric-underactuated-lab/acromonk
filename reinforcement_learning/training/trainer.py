from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, CallbackList
import os
import sys
sys.path.append('../')
from environment.acromonk import AcroMonkEnv
from utils.make_log_folder import make_log_folder
from utils.make_mujoco_simulator import make_mujoco_simulator
from utils.make_rl_model import make_rl_model


def acromonk_training(parameters):
    sim_params = parameters['simulation']
    env_params = parameters['environment']
    train_params = parameters['training']
    rl_model_params = parameters['rl_model']
    reward_params = parameters['reward_setup']

    # make a log folder
    save_folder = make_log_folder()

    # get the simulator
    simulator = make_mujoco_simulator(sim_params)

    # set up the environment
    env = AcroMonkEnv(simulator, env_params, reward_params)
    eval_env = AcroMonkEnv(simulator, env_params, reward_params)

    # get the rl model
    model = make_rl_model(env, rl_model_params)

    # save best model
    eval_freq = train_params["eval_every_steps"]
    eval_callback = EvalCallback(eval_env, best_model_save_path=save_folder,
                                 log_path=save_folder, eval_freq=eval_freq, deterministic=True,
                                 render=False)

    if train_params['save_every_steps'] is not None:
        checkpoint_callback = CheckpointCallback(save_freq=train_params['save_every_steps'],
                                                 save_path=save_folder,
                                                 name_prefix='model')
        callback_list = CallbackList([checkpoint_callback, eval_callback])
        model.learn(total_timesteps=train_params['max_training_steps'], callback=callback_list)
    else:
        model.learn(total_timesteps=train_params['max_training_steps'], callback=eval_callback)

    model.save(os.path.join(save_folder, 'model_final'))

    print('Training finished, all done :)')

