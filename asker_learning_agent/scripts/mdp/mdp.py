TERMINAL_STATE = "END"

class FinishedEpisodeException(Exception):
    """
    """
    def __init__(self):
        self.msg = "The MDP was stepped after an episode terminated"

class MDP():
    """
    Allows you to simulate a basic Markov Decision Process (or MDP). You need
    to provide reward and transition functions, as well as a discount value,
    and terminal state.
    """
    def __init__(self, transition_func, reward_func, discount, terminal_state_test = lambda x: x == TERMINAL_STATE):
        self.transition_func = transition_func
        self.reward_func = reward_func
        self.discount = discount
        self.cumulative_reward = 0
        self.terminal_state_test = terminal_state_test
        self.episode_done = True
        self.curr_state = None

    def step(self, action):
        """
        Moves forward the MDP given an action at the current state. It then
        returns true if the action resulted in a terminal state, and false
        otherwise.
        """
        if self.episode_done or self.curr_state is None:
            raise FinishedEpisodeException()

        new_state = self.transition_func(self.curr_state, action)
        reward = self.reward_func(self.curr_state, action, new_state)

        self.curr_state = new_state
        self.cumulative_reward = reward + self.discount*self.cumulative_reward

        if self.terminal_state_test(self.curr_state):
            self.episode_done = True

        return (reward, self.curr_state)

    def current_state(self):
        """
        Returns the current state
        """
        return self.curr_state

    def is_terminal(self):
        """
        Returns true if the MDP is currently in a terminal state, and false
        otherwise.
        """
        return self.episode_done

    def get_episode_utility(self):
        """
        Get the cumulative reward of the episode up to this point. This is
        calculated as:
        .. math:: \sum_{t=1}^{T}\lambda^t*R_t
        Where T is the current timestep, lambda is the discount factor, and
        R_t is the reward given at timestep t.
        """
        return self.cumulative_reward

    def reset_world(self, initial_state):
        """
        Resets the episote by reseting the state, cumulative reward, and set
        episode done to false.
        """
        self.curr_state = initial_state
        self.cumulative_reward = 0
        self.episode_done = False
