"""
Implements a simple, feed forward fully connected neural network with a sigmoid activation function
for all layers except the last layer (which uses a simple linear activation function s(x)=x). It
also uses a stochastic gradient descent method. This is well suited for value function estimation
in Reinforcement Learning tasks.
"""
import numpy as np

def activation(x):
    """
    Returns the sigmoid activation function for all values in x
    """
    return np.maximum(0.001*x, x)

def d_activation(x):
    """
    Implements the derivative of the sigmoid activation function
    for all values of x
    """
    deriv = np.maximum(0.001*np.sign(x), np.sign(x))
    deriv[deriv == 0] = 1
    return deriv

class NeuralNet:
    """
    Represents a feed forward, fully connected neural network that can be trained using
    stochastic gradient descent. This one makes use of the sigmoid activation function
    for all but the last neuron, which uses a linear activation.
    """
    def __init__(self, sizes, alpha):
        self.sizes = list(sizes)
        self.layers = len(sizes)
        self.alpha = alpha
        self.reset_weights()

    def reset_weights(self):
        """
        Resets all the weights in the neural network to a normal distribution around
        0 with standard deviation of 0.1.
        """
        self.weights = np.array([])
        for i in range(self.layers - 1):
            size = (self.sizes[i] + 1, self.sizes[i+1])
            self.weights = np.append(self.weights,
                                     np.random.normal(scale=0.1, size=size))


    def set_weights(self, weights):
        """
        Set the weights for the neural net. It is assumed that you pass in a numpy
        array of the right size
        """
        self.weights = np.array(weights).flatten().copy()


    def get_layer_weights(self, layer):
        """
        Gets the matrix of weights between a given layer and the next layer from the
        array self.weights. The dimensions are set up so that you can calculate the
        next layer's activation as:
        ```
        last_layer_activation_vector * activation(get_layer_weights(layer))
        ```
        Where last_layer_activation_vector is a row (aka horizontal) vector.
        """
        dims = (self.sizes[layer] + 1, self.sizes[layer+1])
        size = dims[0] * dims[1]
        next_pairs = ((self.sizes[i] + 1, self.sizes[i+1]) for i in range(layer))
        offset = sum(pair[0] * pair[1] for pair in next_pairs)

        weight_subset = self.weights[offset:offset+size].copy()
        weight_subset.resize(dims)

        return np.matrix(weight_subset)

    def get_weights(self):
        """
        Returns all the weights as a flat vector. It is guaranteed to be a copy of
        the weights used internally.
        """
        return self.weights.copy()

    def predict(self, input_x):
        """
        Takes in a given sample (input_x) and calculates the predicted values of
        the output nodes on the neural network.
        """
        a = [input_x.copy()]
        z = [input_x.copy()]

        # Forward propagation
        for layer in range(self.layers - 1):
            bias_node = np.mat(np.ones((a[-1].shape[0], 1)))
            a[-1] = np.insert(a[-1], [a[-1].shape[1]], bias_node, axis=1)
            z += [a[-1] * self.get_layer_weights(layer)]
            a += [activation(z[-1]) if layer != self.layers - 2 else z[-1]]

        return a[-1]

    def gradient(self, training_x):
        """
        Calculates the gradient vector for the neural network of the cost
        function over the training set with respect to the weights.
        """
        # Put training into matrices
        training_x = np.mat(training_x, np.float32)

        # Initialize activation (a) and pre-activation neuron values (z)
        # The invariant holds that a[t] = activation(z[t], t), where
        # activation(*, t) is the activation function for layer t
        a = [training_x.copy()]
        z = [training_x.copy()]

        # Forward propagation
        for layer in range(self.layers - 1):
            bias_node = np.mat(np.ones((a[-1].shape[0], 1)))
            a[-1] = np.insert(a[-1], [a[-1].shape[1]], bias_node, axis=1)
            z += [a[-1] * self.get_layer_weights(layer)]
            a += [activation(z[-1]) if layer != self.layers - 2 else z[-1]]

        # Initialize gradient with all zeros
        grad = [np.mat(np.zeros((self.sizes[layer]+1, self.sizes[layer+1]))) for layer in range(self.layers - 1)]

        # --------------------------------------------------------
        # ---------------- BACK PROPAGATION ----------------------
        # --------------------------------------------------------

        # Iterate over every sample
        for t in range(len(training_x)):
            sample_x = training_x[t]

            # Get sample's activation and pre-activation values
            a_t = [act_layer[t] for act_layer in a]
            z_t = [z_layer[t] for z_layer in z]

            # Calculate errors on last layer
            del_error = np.ones_like(a_t[-1])

            # Go back through each layer
            for layer in range(self.layers - 2, -1, -1):
                layer_weights = self.get_layer_weights(layer)
                # Grab correct activation derivative
                act_deriv = lambda x: (d_activation(x) if layer != self.layers - 2 else np.ones(x.shape))
                # Calculate next layer activation and add bias node
                next_act_d = act_deriv(z_t[layer + 1])

                # Updates for said layer's weight gradients
                grad[layer] += a_t[layer].transpose() * np.multiply(next_act_d, del_error)
                # Update delta relative to activations (aka dC/da[t])
                del_error = np.sum(np.multiply(act_deriv(z_t[layer+1]), del_error[0]) * layer_weights.transpose(), axis=0)
                # Remove the del_error corresponding to the bias neurons
                del_error = np.delete(del_error, del_error.shape[1] - 1, 1)

        # Unroll gradient into a single vector
        grad_unrolled = np.array([])
        for layer_grad in grad:
            layer_unrolled = layer_grad.copy()
            layer_unrolled.resize(layer_grad.shape[0] * layer_grad.shape[1])
            grad_unrolled = np.append(grad_unrolled, layer_unrolled)

        # Ensure this AVERAGES all the training examples (not the sum)
        grad_unrolled /= len(training_x)

        return grad_unrolled
