class Agent:
    def next_action(self, state):
        """
        Method should return the agent's next action given
        the state
        """
        raise NotImplementedError
    
    def action_update(self, reward, new_state):
        """
        Method will be called to update the agent after the
        last action taken by next_action to give new state
        and reward
        """
        raise NotImplementedError
