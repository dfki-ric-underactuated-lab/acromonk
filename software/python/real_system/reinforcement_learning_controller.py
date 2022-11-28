import numpy as np
import os


class RLearningController:
    def __init__(self, weights, biases, activation='tanh'):
        self.weights = weights
        self.expected_input_size = self.weights[0].shape[1]
        self.num_layers = len(self.weights)
        self.biases = biases
        # self.last_obs = np.array([])
        if activation == 'tanh':
            self.activation = np.tanh
        else:
            raise NotImplementedError('Other activation functions than "tanh" not implemented yet')

    def get_torque(self, obs):
        if len(obs) != self.expected_input_size:
            raise ValueError(f'the input you provided does not match '
                             f'the expected vector length {self.expected_input_size}')
        scaler = np.array([3.14/2, 3.14/2, 4.0/2, 4.0/2])
        scaler = scaler[:, np.newaxis]
        
        activation = np.tanh(np.dot(self.weights[0], np.squeeze(obs*scaler)) + self.biases[0])
        for n_layer in range(1, self.num_layers - 1):
            activation = np.tanh(np.dot(self.weights[n_layer], activation) + self.biases[n_layer])
        activation = np.dot(self.weights[-1], activation) + self.biases[-1]

        return activation


def get_controller(load_path='../src/reinforcement_learning/trained_controllers/Right_Left/'):

    weights = np.load(os.path.join(load_path, 'weights_2.npy'), allow_pickle=True)
    biases = np.load(os.path.join(load_path, 'biases_2.npy'), allow_pickle=True)

    controller = RLearningController(weights, biases)

    return controller

