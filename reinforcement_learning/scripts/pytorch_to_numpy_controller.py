# Here check whether the pytorch model and the numpy version give the same torque commands

import os
from utils.paths_handling import project_root_dir
from utils.make_rl_model import load_rl_model
import numpy as np
import torch
import time
import matplotlib.pyplot as plt
from matplotlib import cm



def predict_numpy(obs, weights, biases):
    activation = np.tanh(np.dot(weights[0], obs.T) + biases[0][:, np.newaxis])
    for kk in range(1, len(weights)-1):
         activation = np.tanh(np.dot(weights[kk], activation) + biases[kk][:, np.newaxis])
    # activation = np.tanh(np.dot(weights[1], activation) + biases[1])
    activation = np.dot(weights[-1], activation) + biases[-1][:, np.newaxis]

    return activation.T

rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_7_17_55_16')
rl_checkpoint = 'best'

rl_model, parameters, env = load_rl_model(rl_model_path, rl_checkpoint)

model_pars = rl_model.get_parameters()

policy_weights = []
policy_biases = []
weight_keys = ['mlp_extractor.policy_net.0.weight',
               'mlp_extractor.policy_net.2.weight',
               'action_net.weight']
bias_keys = ['mlp_extractor.policy_net.0.bias',
             'mlp_extractor.policy_net.2.bias',
             'action_net.bias']

policy_weights_np = []
policy_biases_np = []
for weight_key, bias_key in zip(weight_keys, bias_keys):
    policy_weights_np.append(model_pars['policy'][weight_key].numpy())
    policy_biases_np.append(model_pars['policy'][bias_key].numpy())



np.save('weights', policy_weights_np, allow_pickle=True)
np.save('biases', policy_biases_np, allow_pickle=True)
loaded_weights = np.load('weights.npy', allow_pickle=True)
loaded_biases = np.load('biases.npy', allow_pickle=True)

np_predictions = []
time_to_predict = []
loaded_np_predictions = []
rl_model_predictions = []

rand_obs = (2 * np.random.rand(1000, 4) - 1) * np.array([np.pi, np.pi, 4.0, 4.0])[np.newaxis, :]

rl_model_predict = rl_model.policy.predict(torch.tensor(rand_obs), deterministic=True)[0]
np_predict = predict_numpy(rand_obs, policy_weights_np, policy_biases_np)

# plt.figure('Compare pytorch and numpy torque, sanity check')
# plt.plot(np.squeeze(rl_model_predict), np.squeeze(rl_model_predict), 'r-', zorder=0)
# plt.scatter(np.squeeze(rl_model_predict), np.squeeze(np_predict), 2, zorder=1)
# plt.xlabel('pytorch model torque')
# plt.ylabel('numpy model torque')
# plt.show()
# X, Y = np.meshgrid(rand_obs[:, 2], rand_obs[:, 3])
# plt.figure('What is the controller actually doing?')
# plt.pcolor(X, Y, rl_model_predict)
# plt.show()

X, Y = np.meshgrid(rand_obs[:, 2], rand_obs[:, 3])
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(X, Y, rl_model_predict, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
plt.show()

a = None

# for k in range(10):
#     rand_obs = np.random.rand(4)
#     time_pre_predict = time.time()
#     np_predict = predict_numpy(rand_obs, policy_weights_np, policy_biases_np)
#     time_post_predict = time.time()
#     time_to_predict.append(time_post_predict - time_pre_predict)
#     loaded_np_predict = predict_numpy(rand_obs, loaded_weights, loaded_biases)
#     rl_model_predict = rl_model.policy.predict(torch.tensor(rand_obs), deterministic=True)[0]
#
#     np_predictions.append(np_predict[0])
#     loaded_np_predictions.append(loaded_np_predict[0])
#     rl_model_predictions.append(rl_model_predict[0])
#
# print(f'Predictions stable_baselines:{rl_model_predictions}')
# print(f'Predictions numpy:{np_predictions}')
# print(f'Predictions numpy loaded:{loaded_np_predictions}')
# print(f'Predictions time numpy:{time_to_predict}')

a = None



