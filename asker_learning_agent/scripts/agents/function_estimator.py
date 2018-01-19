from mdp.states import state_action_reducer, state_action_reduced_range
from nn.neuralnet import NeuralNet
import numpy as np

class WrongWeightVectorSizeException(BaseException):
    def __init__(self):
        pass

class FunctionEstimator:
    def get_qval(self, state, action):
        raise NotImplementedError

    def get_grad(self, state, action):
        raise NotImplementedError

    def set_weights(self, weights):
        raise NotImplementedError
    
    def get_weights(self):
        raise NotImplementedError

class NeuralNetEstimator(FunctionEstimator):
    def __init__(self, sample_state, sample_action, hidden_layer_sizes):
        sample_input = state_action_reducer(sample_state, sample_action)
        nn_sizes = [sample_input.shape[1]] + hidden_layer_sizes + [1]

        self.nn = NeuralNet(nn_sizes, 0.9) # Alpha parameter is ignored

    def get_qval(self, state, action):
        input_form = state_action_reducer(state, action)
        return self.nn.predict(input_form)[0, 0]

    def get_grad(self, state, action):
        input_form = state_action_reducer(state, action)
        return self.nn.gradient(input_form)

    def set_weights(self, weights):
        self.nn.set_weights(weights)

    def get_weights(self):
        return self.nn.get_weights()

class FourierEstimator(FunctionEstimator):
    def __init__(self, sample_state, sample_action, num_series):
        sample_input = state_action_reducer(sample_state, sample_action)

        # Each range has double the length, so that we can represent the function using
        # only cosine features.
        self.input_ranges = [(x[0], 2*x[1] - x[0]) for x in state_action_reduced_range()]
        
        self.dims = sample_input.shape[1]
        self.series = num_series

        self.reset_weights()
    
    def reset_weights(self):
        # Only cos(x) features are used, with periods 2*range*n on each dimension
        num_weights = self.series * self.dims
        self.weights = np.array([])
        for i in range(num_weights):
            self.weights = np.append(self.weights,
                                     np.random.normal(scale=0.1))

    def get_weight(self, d, s):
        # Weight index in dimension-major order
        return self.weights[s + d*self.series]

    def set_weight(self, d, s, val):
        # Weight index in dimension-major order
        self.weights[s + d*self.series] = val

    def get_qval(self, state, action):
        input_form = state_action_reducer(state, action)

        val = 0

        for d in range(self.dims):
            # Each feature is f_i(x) = cos(2*x*pi*v / T\pi)
            # Period is length of corresponding self.input_ranges
            period = self.input_ranges[d][1] - self.input_ranges[d][0]
            # Shift range down from [start, start + len] to [0, len]
            x = input_form[0, d] - self.input_ranges[d][0]

            for s in range(self.series):
                val += self.get_weight(d, s) * np.cos(2 * np.pi * s * x / period)
        
        return val


    def get_grad(self, state, action):
        input_form = state_action_reducer(state, action)

        grad = np.zeros_like(self.weights)

        for d in range(self.dims):
            # Each feature is f_i(x) = cos(2*x*pi*v / T\pi)
            # Period is length of corresponding self.input_ranges
            period = self.input_ranges[d][1] - self.input_ranges[d][0]
            # Shift range down from [start, start + len] to [0, len]
            x = input_form[0, d] - self.input_ranges[d][0]

            for s in range(self.series):

                grad[s + d*self.series] = np.cos(2 * np.pi * s * x / period)
        return grad

    def set_weights(self, weights):
        if type(weights == np.ndarray):
            new_weights = weights.copy()
        else:
            new_weights = np.array(weights)

        if new_weights.size != self.dims * self.series:
            raise WrongWeightVectorSizeException

        self.weights = new_weights

    def get_weights(self):
        return self.weights.copy()
