import gym
from gym import spaces
import numpy as np
import inspect
from mujoco_py import cymj, MjSimState


class AcroMonkEnv(gym.Env):
    """Custom environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, simulator, env_params, reward_params):
        super(AcroMonkEnv, self).__init__()

        self.sim = simulator
        self.target_configuration = env_params['target_configuration']
        self.target_end_effector_coordinates = env_params['target_pos_end_effector']
        self.target_end_effector_coordinates_dx = env_params['target_pos_end_effector_dx']
        self.torque_lims = np.array(env_params['torque_lims'])
        self.control_mode = env_params["control_mode"]
        self.height_threshold_end_effector = env_params['height_threshold_end_effector']
        self.kp = env_params['kp']
        self.kd = env_params['kd']
        self.max_steps_per_episode = env_params['max_steps_per_episode']
        self.target_configuration_dq = env_params['target_configuration_dq']
        self.target_configuration_dq_dot = env_params['target_configuration_dq_dot']

        self.reward_function_list = self._get_reward_functions(reward_params)

        action_lim_min = env_params[f'action_lim_min_{self.control_mode}']
        action_lim_max = env_params[f'action_lim_max_{self.control_mode}']

        self.init_state = env_params['init_state']
        self.init_state_rand_rand_range = env_params['init_state_rand_range']

        self.action_space = spaces.Box(low=np.array(action_lim_min),
                                           high=np.array(action_lim_max))
        self.observation_space = spaces.Box(low=np.array(env_params["obs_lim_min"]),
                                            high=np.array(env_params["obs_lim_max"]))

        self.obs_noise_range = np.zeros(np.array(action_lim_min).shape)
        if "obs_noise_range" in env_params.keys():
            self.obs_noise_range = env_params['obs_noise_range']

        # to be filled during simulation
        self.current_config = None
        self.current_end_effector_pos = None
        self.last_end_effector_pos = None
        self.last_action = None
        self.last_applied_torque = None
        self.steps_taken_this_ep = None
        self.reached_target = False
        self.stick_contact = False

        self.reset()

    def step(self, action):

        torque = self._get_torque(action)
        self.last_applied_torque = torque
        self.sim.data.ctrl[:] = torque
        self.sim.step()

        self.steps_taken_this_ep += 1

        self.current_config = np.concatenate((self.sim.get_state()[1], self.sim.get_state()[2]))
        self.current_end_effector_pos = self.sim.data.get_body_xpos('hook_2_tip')

        done = False

        # get the reward here
        observation = self._get_observation()
        reward = self._calculate_reward(observation, action)
        if self.reached_target:
            reward += 30.0

        self.last_action = action
        self.last_end_effector_pos = self.current_end_effector_pos

        # check for termination
        if (self.steps_taken_this_ep >= self.max_steps_per_episode) or self.stick_contact or self.reached_target:
            done = True

        info = {}

        return observation, reward, done, info

    def reset(self):
        self.current_config = None
        self.current_end_effector_pos = None
        self.last_end_effector_pos = None
        self.last_action = None
        if self.control_mode == 'T':
            self.last_action = 0.0
        self.steps_taken_this_ep = None
        self.reached_target = False
        self.stick_contact = False
        self.last_applied_torque = 0.0

        self.sim.reset()
        # cymj.set_pid_control(self.sim.model, self.sim.data)
        sim_state = self.sim.get_state()

        init_state_with_rand = np.array(self.init_state)
        init_state_with_rand += (2*(np.random.rand(*init_state_with_rand.shape)) - 1) * self.init_state_rand_rand_range

        start_state = MjSimState(0.0, init_state_with_rand[:2], init_state_with_rand[2:],
                                 sim_state.act, sim_state.udd_state)
        self.sim.set_state(start_state)
        self.sim.forward()

        # reset records
        self.steps_taken_this_ep = 0
        self.last_action = None
        self.current_end_effector_pos = self.sim.data.get_body_xpos('hook_2_tip')
        self.last_end_effector_pos = self.sim.data.get_body_xpos('hook_2_tip')
        self.current_config = np.concatenate((self.sim.get_state()[1], self.sim.get_state()[2]))

        observation = self._get_observation()

        return observation

    def render(self, mode="human"):
        pass

    def _get_reward_functions(self, rewards_and_weight_dict):
        # a function to collect the known reward functions
        reward_functions_list = []
        all_class_methods = inspect.getmembers(self, predicate=inspect.ismethod)
        known_reward_functions = [cm for cm in all_class_methods if cm[0].startswith('_reward')]
        known_reward_names = [krf[0] for krf in known_reward_functions]
        for reward_name, reward_weight in rewards_and_weight_dict.items():
            reward_function_idx = known_reward_names.index(f'_reward_{reward_name}')
            weighted_reward_function = lambda obs, act, reward_weight=reward_weight, reward_function_idx=reward_function_idx: reward_weight * known_reward_functions[reward_function_idx][1](obs, act)
            reward_functions_list.append(weighted_reward_function)

        return reward_functions_list

    def _calculate_reward(self, observations, action):
        rew = 0
        for reward_function in self.reward_function_list:
            rew += reward_function(observations, action)
        return rew

    # reward functions that can be used
    def _reward_target_configuration_q(self, observation, action):
        rew = 0
        if self.target_configuration is not None:
            current_dist_to_target = np.abs(self.current_config[:2] - self.target_configuration[:2])
            if (current_dist_to_target < self.target_configuration_dq).all():
                rew = 1.0
                self.reached_target = True
                print('Reached target configuration!!')

        return rew

    def _reward_target_configuration_q_dot(self, observation, action):
        rew = 0
        if self.target_configuration is not None:
            current_dist_to_target = np.abs(self.current_config[2:] - self.target_configuration[2:])
            if (current_dist_to_target < self.target_configuration_dq_dot).all():
                rew = 1.0
                # self.reached_target = True

        return rew

    def _reward_target_configuration_distance_q(self, observation, action):
        rew = 0

        if self.target_configuration is not None:
            rew = np.exp(-np.mean(np.square(self.current_config[:2] - self.target_configuration[:2])))
            # rew = np.sum(np.square(self.current_config[:2] - self.target_configuration[:2]))

        return rew

    def _reward_get_away_from_start_q2(self, observation, action):
        current_q2 = self.current_config[1]
        rew_cutoff = 0.0
        # rew = -(1/60) * ((current_q2 - self.init_state[1])**3 - (2.0 - self.init_state[1])**3)
        rew = np.abs(current_q2 - self.init_state[1]) / np.abs(self.init_state[1])
        if current_q2 > rew_cutoff:
            rew = 1.0
        if current_q2 < self.init_state[1]:
            rew = 0.0

        return rew

    def _reward_target_end_effector_position(self, observation, action):
        rew = 0
        if self.target_end_effector_coordinates is not None:
            dist_to_target = np.abs(self.current_end_effector_pos[1:] - self.target_end_effector_coordinates[1:])
            if (dist_to_target < self.target_end_effector_coordinates_dx[1:]).all():
                rew = 1
                # print('Reached target end effector position!!!')

        return rew

    def _reward_end_effector_too_low(self, observation, action):
        rew = 0.0
        if self.current_end_effector_pos is not None:
            z_coord_end_effector = self.current_end_effector_pos[2]
            if z_coord_end_effector < self.height_threshold_end_effector:
                rew = 1.0

        return rew

    def _reward_end_effector_forward_motion(self, observation, action):
        rew = 0.0
        if self.last_end_effector_pos is not None:
            y_coord_end_effector = self.current_end_effector_pos[1]
            y_coord_last_end_effector = self.last_end_effector_pos[1]
            if y_coord_last_end_effector < y_coord_end_effector:
                rew = 1.0

        return rew

    def _reward_action_jerkiness(self, observation, action):
        rew = 0
        # only calculate meaningful reward if at least one action has been taken
        if self.last_action is not None:
            rew = np.sum(np.square(self.last_action - action))
        return rew

    def _reward_stick_contact(self, observation, action):
        contacts = self.sim.data.contact
        contacts_list_1 = [self.sim.model.geom_id2name(c.geom1) for c in contacts]
        contacts_list_2 = [self.sim.model.geom_id2name(c.geom2) for c in contacts]
        contact_participant_1 = set(contacts_list_1)
        contact_participant_2 = set(contacts_list_2)

        set_contact_meshes = contact_participant_1.union(contact_participant_2)
        full_stick_set = {'stick_start', 'stick_base', 'stick_target'}
        target_stick_set = {'stick_target'}
        rew = 0.0
        if len(full_stick_set.intersection(set_contact_meshes)) > 0:
            self.stick_contact = True
            rew = 1.0

            if len(target_stick_set.intersection(set_contact_meshes)):
                # Ok, now have to look deeper into the contacts
                if 'stick_target' in contacts_list_1:
                    contact_idcs_target_stick_1 = [contacts_list_1.index('stick_target')]
                else:
                    contact_idcs_target_stick_1 = []
                if 'stick_target' in contacts_list_2:
                    contact_idcs_target_stick_2 = [contacts_list_2.index('stick_target')]
                else:
                    contact_idcs_target_stick_2 = []

                contact_idcs_target = contact_idcs_target_stick_1 + contact_idcs_target_stick_2

                for idx in contact_idcs_target:
                    contact_target = contacts[idx]

                    other_contact_geom = self.sim.model.geom_id2name(contact_target.geom1)
                    if other_contact_geom == 'stick_target':
                        other_contact_geom = self.sim.model.geom_id2name(contact_target.geom2)

                    if other_contact_geom in ['hook_2_upper', 'hook_2_lower', 'hook_2_upper_tip']:
                        if (self.current_end_effector_pos[2] > 0) and (self.current_end_effector_pos[1] < 0.38):
                            rew = 0.0
                            print('Grabbed target stick!!')
                            print('Reversing contact penalty')
                            self.stick_contact = False
                            self.reached_target = True
        return rew

    def _reward_target_end_effector_distance(self, observation, action):
        rew = 0
        if self.current_end_effector_pos is not None:
            rew = np.sum(np.square(self.current_end_effector_pos[1:] - self.target_end_effector_coordinates[1:]))
        return rew

    def _reward_target_configuration_distance(self, observation, action):
        rew = np.sum(np.square(self.current_config - self.target_configuration))
        return rew

    def _reward_intermediate_q_waypoint(self, observation, action):
        rew = 0.0
        if (0.7 < self.current_config[0] < 0.9) and (2.0 < self.current_config[1] < 2.2):
            rew = 1.0
        return rew

    def _reward_torque_penalty(self, observation, action):
        rew = 0.0
        torque_thres = 3.0
        if self.last_applied_torque is not None:
            abs_torque = np.abs(self.last_applied_torque)
            if abs_torque > torque_thres:
                rew = ((abs_torque - torque_thres)**2)
        return rew

    def _reward_velocity_penalty(self, observation, action):
        rew = 0.0
        vel_thres = 8
        current_elbow_vel = self.current_config[3]
        abs_vel = np.abs(current_elbow_vel)
        if abs_vel > vel_thres:
            rew = (abs_vel - vel_thres)**2
        return rew

    def _reward_end_effector_below_bar_or_further(self, observation, action):
        rew = 0.0
        x_thres = 0.4
        curr_x_pos = self.current_end_effector_pos[1]
        curr_y_pos = self.current_end_effector_pos[2]
        if curr_x_pos > x_thres and curr_y_pos < 0.0:
            rew = np.square((curr_x_pos - x_thres) * 10)

        return rew

    def _reward_full_task_space(self, observation, action):
        x = self.current_end_effector_pos[1:]
        rew = self._two_d_polynomial_reward_source(np.array([-0.34, 0]), x, mode="sink", d_max=0.10)
        rew += self._two_d_polynomial_reward_source(np.array([0, 0]), x, mode="sink", d_max=0.17)
        # rew += self._two_d_polynomial_reward_source(np.array([0.34, -0.2]), x, mode="sink", d_max=0.15)
        rew += self._two_d_polynomial_reward_source(np.array([0.346783364, 0.022286]), x, mode="source", d_max=0.3,
                                                    linear_separatrix={'a': 1.7, 'b': 0.00}, separatrix_condition='higher')
        rew += self._two_d_polynomial_reward_source(np.array([0.346783364, 0.022286]), x, mode="source", d_max=0.15,
                                                    linear_separatrix={'a': 1.7, 'b': 0.00},
                                                    separatrix_condition='higher')
        rew += self._two_d_polynomial_reward_source(np.array([0.346783364, 0.022286]), x, mode="source", d_max=0.1,
                                                    linear_separatrix={'a': 1.7, 'b': 0.00},
                                                    separatrix_condition='higher')
        rew += 5 * self._two_d_polynomial_reward_source(np.array([0.34, 0]), x, mode="sink", d_max=0.15,
                                                    linear_separatrix={'a': 1.7, 'b': 0.00}, separatrix_condition='lower')

        return rew

    # util functions
    def _get_torque(self, action, externally_defined_config=None):
        if externally_defined_config is not None:
            config = externally_defined_config
        else:
            config = self.current_config

        if self.control_mode == 'T':
            torque = action[0]
        elif self.control_mode == 'PD':
            torque = self.kp * (config[1] - action[0]) + self.kd * (config[3] - action[1])
        else:
            raise ValueError(f'Unknown control mode {self.control_mode}')

        return np.clip(torque, self.torque_lims[0], self.torque_lims[1])

    def _get_observation(self, otherwise_defined_config=None):
        if otherwise_defined_config is None:
            current_config = self.current_config
        else:
            current_config = otherwise_defined_config
        current_config += np.random.randn(len(current_config)) * self.obs_noise_range
        return current_config

    def _two_d_polynomial_reward_source(self, x_origin, x_now, mode='source', d_max=0.2, power=2,
                                        linear_separatrix=None, separatrix_condition='lower'):
        dist_to_source = np.linalg.norm(x_origin - x_now)
        rew = 0
        if dist_to_source < d_max:
            rew = ((dist_to_source - d_max) / d_max) ** power

        if mode == 'sink':
            rew *= -1

        if linear_separatrix is not None:
            a = linear_separatrix['a']
            b = linear_separatrix['b']

            x_rel = x_now - x_origin

            if separatrix_condition == 'lower':
                if x_rel[1] > a * x_rel[0] + b:
                    rew = 0

            elif separatrix_condition == 'higher':
                if x_rel[1] < a * x_rel[0] + b:
                    rew = 0

        return rew
