from mujoco_py import load_model_from_xml, load_model_from_path, MjSim, MjSimState, MjViewer, MjViewerBasic
import os
import numpy as np
import matplotlib.pyplot as plt
import glfw
# import cv2
from utils.paths_handling import project_root_dir
from utils.make_rl_model import load_rl_model
import pandas



# This is really good!!!
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_5_15_16_32')
# rl_checkpoint = 'best'

rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_7_17_55_16')
rl_checkpoint = 'best'

# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_7_20_21_51')
# rl_checkpoint = 'best'

#
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_5_13_49_23')
# rl_checkpoint = '480000'

# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_2_11_32_21')
# rl_checkpoint = 'best'
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_1_15_49_26')
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_9_1_14_27_9')
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_8_29_14_13_16')
# '412000'
# rl_checkpoint = '100000'
# rl_checkpoint = 'best'
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_8_30_17_38_41')
# rl_model_path = os.path.join(project_root_dir, 'data', 'trained_agents', '2022_8_30_17_9_4')
# rl_checkpoint = 'best'

model = load_model_from_path(os.path.join(project_root_dir, 'data', 'robot_model', '20220510_ShwingBotV1.xml'))
# model = load_model_from_path("../../../../data/urdf/RH5v2.mjb")
model.opt.timestep = 0.004
sim = MjSim(model)
start_time_buffer = 100

# viewer = MjViewer(sim)
viewer = MjViewerBasic(sim)
# viewer.cam.trackbodyid = 1
viewer.cam.distance = model.stat.extent * 0.80
viewer.cam.lookat[2] += .3
viewer.cam.elevation = -10
viewer.cam.azimuth = -180

window_size = (1920, 1080)
glfw.set_window_size(viewer.window, window_size[0], window_size[1])

# get controller
rl_model, parameters, env = load_rl_model(rl_model_path, rl_checkpoint)

init_state = np.array(parameters['environment']['init_state'])

sim_state = sim.get_state()
# sim.set_state(start_state)
# this is the initial state
start_state = MjSimState(0.0, init_state[:2], init_state[2:],
                         sim_state.act, sim_state.udd_state)

sim.set_state(start_state)

sim_steps = parameters['environment']['max_steps_per_episode']
torque_recorded = np.zeros(sim_steps)
data_recorded = np.zeros((8, sim_steps))
reward = 0
i_max = sim_steps
for i in range(sim_steps + 50):
    env.sim.reset()  # range(parameters['environment']['max_steps_per_episode']):
    current_sim_state = sim.get_state()
    current_state = np.concatenate((current_sim_state[1], current_sim_state[2]))
    observation = env._get_observation(otherwise_defined_config=current_state)
    action = rl_model.policy.predict(observation, deterministic=True)[0]
    torque = env._get_torque(action, externally_defined_config=current_state)
    if i < sim_steps:
        torque_recorded[i] = torque

    env.current_config = np.concatenate((current_sim_state[1], current_sim_state[2]))
    env.current_end_effector_pos = sim.data.get_body_xpos('hook_2_tip')
    env_sim_state = env.sim.get_state()
    target_state = MjSimState(env_sim_state.time, env.current_config[:2], env.current_config[2:],
                              env_sim_state.act, env_sim_state.udd_state)
    env.sim.set_state(target_state)
    env.sim.forward()
    reward += env._calculate_reward(1, 1)

    sim.data.ctrl[:] = torque
    sim.step()
    viewer.render()

    ee_pos = sim.data.get_body_xpos('hook_2_tip')
    if i < sim_steps:
        data_recorded[:2, i] = current_sim_state[1]
        data_recorded[2:4, i] = current_sim_state[2]
        data_recorded[4:7, i] = ee_pos
        data_recorded[7, i] = torque

    if env.reached_target:
        i_max = i
        break

    if env.stick_contact:
        a = None
        i_max = i
        break

print(f'Cumulative Reward: {reward}')


data_recorded = data_recorded[:, :i_max]
# save recorded data
data_save_path = os.path.join(rl_model_path, 'eval_run_data')
np.save(data_save_path, data_recorded)

t = np.arange(sim_steps) * 0.004
t = t[:i_max]

plt.figure()
plt.subplot(5, 1, 1)
plt.plot(t, data_recorded[0])
plt.ylabel('theta 1 [rad]')

plt.subplot(5, 1, 2)
plt.plot(t, data_recorded[1])
plt.ylabel('theta 2 [rad]')

plt.subplot(5, 1, 3)
plt.plot(t, data_recorded[2])
plt.ylabel('theta 1 dot [rad/s]')

plt.subplot(5, 1, 4)
plt.plot(t, data_recorded[3])
plt.ylabel('theta 2 dot [rad/s]')

plt.subplot(5, 1, 5)
plt.plot(t, data_recorded[7])
plt.ylabel('torque [Nm]')

plt.xlabel('time [s]')
plt.show()

dt_data_des = 0.001
t_data_des = np.arange(int(t[-1] / dt_data_des) + 1) * dt_data_des

pos_vel_data_interp = np.zeros((4, len(t_data_des)))
for k in range(4):
    pos_vel_data_interp[k, :] = np.interp(t_data_des, t, data_recorded[k, :])


data_with_time = np.row_stack((t_data_des[np.newaxis, :], pos_vel_data_interp)).T

columns = ['time', 'shoulder_pos', 'elbow_pos', 'shoulder_vel', 'elbow_vel']

additional_columns = ['shoulder_acc', 'shoulder_jerk', 'shoulder_torque', 'elbow_acc', 'elbow_jerk', 'elbow_torque',
                      'elbow_torque_tvlqr', 'K1', 'K2', 'K3', 'K4', 'k0']
additional_data = np.random.rand(data_with_time.shape[0], len(additional_columns))

data_with_additional = np.hstack((data_with_time[:, :5], additional_data))

df = pandas.DataFrame(data_with_additional, columns=columns + additional_columns)

df.to_csv(os.path.join(rl_model_path, f'traj_RLearn_checkpoint_{rl_checkpoint}.csv'), index=False)

a = None
print('All done!')
