# This script loads the pretrained BF controller and runs it in simulation

# system imports
from mujoco_py import load_model_from_xml, load_model_from_path, MjSim, MjSimState, MjViewer, MjViewerBasic
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
# local imports
sys.path.append('../')
from utils.paths_handling import project_root_dir
from utils.make_rl_model import load_rl_model

# load pretrained control policy network
rl_model_path = os.path.join(project_root_dir, 'data', 'trained_controllers')
rl_checkpoint = 'best'
rl_model, parameters, env = load_rl_model(rl_model_path, rl_checkpoint)

# load robot simulation model
model = load_model_from_path(os.path.join(project_root_dir, 'data', 'simulation_models', 'acromonk_mujoco.xml'))
model.opt.timestep = parameters['simulation']['dt']
sim = MjSim(model)
start_time_buffer = 100

# setup simulation
viewer = MjViewerBasic(sim)
viewer.cam.distance = model.stat.extent * 0.80
viewer.cam.lookat[2] += .3
viewer.cam.elevation = -10
viewer.cam.azimuth = -180

init_state = np.array(parameters['environment']['init_state'])

sim_state = sim.get_state()
start_state = MjSimState(0.0, init_state[:2], init_state[2:],
                         sim_state.act, sim_state.udd_state)

sim.set_state(start_state)

sim_steps = parameters['environment']['max_steps_per_episode']
torque_recorded = np.zeros(sim_steps)
data_recorded = np.zeros((8, sim_steps))
reward = 0
i_max = sim_steps

# simulation loop
for i in range(sim_steps + 50):
    current_sim_state = sim.get_state()
    current_state = np.concatenate((current_sim_state[1], current_sim_state[2]))
    observation = env._get_observation(otherwise_defined_config=current_state)
    action = rl_model.policy.predict(observation, deterministic=True)[0]
    torque = env._get_torque(action, externally_defined_config=current_state)

    sim.data.ctrl[:] = torque
    sim.step()
    viewer.render()

    ee_pos = sim.data.get_body_xpos('hook_2_tip')
    if i < sim_steps:
        data_recorded[:2, i] = current_sim_state[1]
        data_recorded[2:4, i] = current_sim_state[2]
        data_recorded[4:7, i] = ee_pos
        data_recorded[7, i] = torque

# plotting
t = np.arange(sim_steps) * 0.004
t = t[:i_max]

fig = plt.figure(figsize=(6, 10))
plt.subplot(5, 1, 1)
plt.plot(t, data_recorded[0])
plt.ylabel(r'$q_{1}$ [rad]')

plt.subplot(5, 1, 2)
plt.plot(t, data_recorded[1])
plt.ylabel(r'$q_{2}$ [rad]')

plt.subplot(5, 1, 3)
plt.plot(t, data_recorded[2])
plt.ylabel(r'$\dot{q}_{1}$ [rad/s]')

plt.subplot(5, 1, 4)
plt.plot(t, data_recorded[3])
plt.ylabel(r'$\dot{q}_{2}$ [rad/s]')

plt.subplot(5, 1, 5)
plt.plot(t, data_recorded[7])
plt.ylabel(r'$u$ [Nm]')

plt.xlabel('time [s]')
fig.align_ylabels()
plt.show()

print('All done!')
