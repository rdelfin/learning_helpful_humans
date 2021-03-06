from agents.agent import Agent
from firebase_scripts import travel_data
from agents.function_estimator import FunctionEstimator
from mdp.states import Action, FullState, is_terminal_state

from functools import reduce
import random

import numpy as np

import rospy

class SarsaAgent(Agent):
    def __init__(self, eps, alpha, gamma, estimator):
        # FIX: Stop depending on states class to define sizes
        # 9 input neurons:
        #   1 for time
        #   7 for each day of week
        #   num_actions for each action (move to location, excluding the current location)
        #   1 for whether the action involves asking or not
        self.possible_actions = [Action(loc, ask) for loc in travel_data.get_location_ids() for ask in [True, False]]
        self.eps = eps
        self.alpha = alpha
        self.gamma = gamma
        self.estimator = estimator
        self.future_state_actions = []
        self.past_state_actions = []

    def generate_next_action(self, state):
        max_action = max(self.possible_actions, key=lambda action: self.estimator.get_qval(state, action))
        rand_action = random.choice(list(self.possible_actions))

        action_taken = rand_action if random.random() < self.eps else max_action   # Epsilon-greedy behaviour

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
