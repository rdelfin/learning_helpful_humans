from firebase_scripts import travel_data
from mdp.states import Action
from agents.agent import Agent
import random


class RandomAgent(Agent):
    def __init__(self):
        self.travel_locations = travel_data.get_location_ids()

    def next_action(self, state):
        return Action(True, random.choice(list(self.travel_locations)))
    
    def action_update(self, reward, new_state):
        pass
