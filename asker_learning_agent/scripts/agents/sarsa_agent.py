from agents.agent import Agent
from firebase_scripts import travel_data
from agents.function_estimator import FunctionEstimator
from mdp.states import Action, FullState, is_terminal_state

from functools import reduce
import random

import numpy as np

class SarsaAgent(Agent):
    def __init__(self, eps, alpha, gamma, estimator):
        # FIX: Stop depending on states class to define sizes
        # 8 input neurons:
        #   1 for time
        #   7 for each day of week
        #   num_actions for each action (move to location, excluding the current location)
        self.travel_locations = travel_data.get_location_ids()
        self.num_actions = len(self.travel_locations) - 1
        self.eps = eps
        self.alpha = alpha
        self.gamma = gamma
        self.estimator = estimator
        self.future_state_actions = []
        self.past_state_actions = []

    def generate_next_action(self, state):
        max_location = max(self.travel_locations, key=lambda loc: self.estimator.get_qval(state, Action(True, loc)))
        rand_location = random.choice(list(self.travel_locations))

        location_taken = rand_location if random.random() < self.eps else max_location   # Epsilon-greedy behaviour
        action_taken = Action(True, location_taken)

        self.future_state_actions += [(state, action_taken)]

    def next_action(self, state):
        if(not self.future_state_actions):
            self.generate_next_action(state)

        new_action = self.future_state_actions[0][1]
        self.past_state_actions += [self.future_state_actions[0]]
        del self.future_state_actions[0]
        return new_action

    def action_update(self, reward, new_state):
        # Implemented semi-gradient SARSA update
        # (page 198 of Sutton http://www.incompleteideas.net/book/bookdraft2017nov5.pdf)
        if is_terminal_state(new_state):
            qval_diff = reward - self.estimator.get_qval(*self.past_state_actions[-1])
        else:
            self.generate_next_action(new_state)
            next_sa_pair = self.future_state_actions[0]
            qval_diff = reward + self.gamma * self.estimator.get_qval(*next_sa_pair)

        self.estimator.set_weights(self.estimator.get_weights() +
            self.estimator.get_grad(*self.past_state_actions[-1]) * self.alpha * qval_diff)
