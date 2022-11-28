from abc import ABC, abstractmethod
import numpy as np
import os


class AbstractSwingController(ABC):
    @abstractmethod
    def __init__(self):
        self.controller_type = None

    @abstractmethod
    def get_control_command(self, meas_state, t, brach_state):
        tau_cmd = None
        kp_scale = None
        kd_scale = None
        des_traj_vals = None
        controller_specific_info = None

        return tau_cmd, des_traj_vals, kp_scale, kd_scale, controller_specific_info


class RLearningController(AbstractSwingController):
    def __init__(self, load_path='../src/reinforcement_learning/trained_controllers/Right_Left/', activation='tanh'):
        self.controller_type = 'rl'
        weights, biases = self._load_data(load_path)
        self.weights = weights
        self.biases = biases
        self.kp_scale = 0.0
        self.kd_scale = 0.0
        self.tau_scale = 1.0
        self.target_q = np.array([0.73, 1.97])
        self.target_dq = np.array([0.05, 0.05])
        self.switched_off_controller = False
        self.zero_offset_sh = np.deg2rad(0.0)  # We found out that the zero offset is actually not needed
        self.torque_scaler_odd = 0.90  # 0.92
        self.torque_scaler_even = 1.00  # 1.07
        self.des_traj_vals = {'el_des_pos': 0.0,
                              'el_des_vel': 0.0,
                              'sh_des_pos': np.NAN,
                              'sh_des_vel': np.NAN}

        self.expected_input_size = self.weights[0].shape[1]
        self.num_layers = len(self.weights)
        # self.last_obs = np.array([])
        if activation == 'tanh':
            self.activation = np.tanh
        else:
            raise NotImplementedError('Other activation functions than "tanh" not implemented yet')

    @staticmethod
    def _load_data(load_path='../src/reinforcement_learning/trained_controllers/Right_Left/'):
        weights = np.load(os.path.join(load_path, 'weights_2.npy'), allow_pickle=True)
        biases = np.load(os.path.join(load_path, 'biases_2.npy'), allow_pickle=True)

        return weights, biases

    def get_control_command(self, obs, t, brach_sign):
        # processing observation
        obs[0] += self.zero_offset_sh
        if brach_sign == -1:
            obs = np.multiply(obs, [[1], [-1], [1], [-1]])
        if len(obs) != self.expected_input_size:
            raise ValueError(f'the input you provided does not match '
                             f'the expected vector length {self.expected_input_size}')

        # if not self.switched_off_controller:
        scaler = np.array([3.14 / 2, 3.14 / 2, 4.0 / 2, 4.0 / 2])
        # scaler = np.array([3.14, 3.14, 4.0 / 2.0, 4.0 / 2.0])
        scaler = scaler[:, np.newaxis]
        scaled_obs = np.squeeze(obs * scaler)
        # scaled_obs = np.clip(scaled_obs, -1.0, 1.0)
        activation = np.tanh(np.dot(self.weights[0], scaled_obs) + self.biases[0])
        for n_layer in range(1, self.num_layers - 1):
            activation = np.tanh(np.dot(self.weights[n_layer], activation) + self.biases[n_layer])
        tau_cmd = np.dot(self.weights[-1], activation) + self.biases[-1]
        tau_cmd *= brach_sign
        if brach_sign == 1:
            tau_cmd *= self.torque_scaler_odd
        elif brach_sign == -1:
            tau_cmd *= self.torque_scaler_even
        '''
        else:
            tau_cmd = 0.0
            print(f'{obs} target configuration {self.target_q} with within error range {self.target_dq}, '
                  f'commanding torque {tau_cmd}')
        '''

        # post-processing torque command
        # tau_cmd = np.clip(tau_cmd, -1.0, 1.0)
        # tau_cmd *= self.tau_scale

        return tau_cmd, self.des_traj_vals, self.kp_scale, self.kd_scale, {}

