from functools import reduce
import random

from firebase_scripts import travel_data, schedule_data
import util

import numpy as np
import scipy
import scipy.stats

# Constants: replace with values in person tables
arrival_stddev = 120
departure_stddev = 180
open_stddev = 300

class Request:
    def __init__(self, *, location, time_of_day, day_of_week):
        self.location = location
        self.time_of_day = time_of_day
        self.day_of_week = day_of_week
    
    def __repr__(self):
        return "Request(location=%s, time_of_day=%d, day_of_week=%d)" % \
                (self.location, self.time_of_day, self.day_of_week)

class Action:
    def __init__(self, travel=False, location=None):
        self.travel = travel
        self.location = location
    
    def __repr__(self):
        return "Action(travel=%s, location=%s)" % (str(self.travel), self.location)

def is_terminal_state(state):
    return state.time_of_day > 64800  # 6pm in seconds

class FullState:
    def __init__(self, *, location, time_of_day, day_of_week, person_present_map, request_history):
        self.location = location
        self.time_of_day = time_of_day
        self.day_of_week = day_of_week
        self.person_present_map = person_present_map
        self.request_history = request_history

    def time_only_state(self):
        return (self.time_of_day, self.day_of_week)

    def time_and_location_state(self):
        return (self.time_of_day, self.day_of_week, self.location)

    def __repr__(self):
        return "FullState(location=%s, time_of_day=%d, day_of_week=%d," \
                "person_present_map=%s)" % \
                (self.location, self.time_of_day, self.day_of_week, str(self.person_present_map))

def state_action_reducer(state, action):
    travel_locations = travel_data.get_location_ids()
    num_actions = len(travel_locations)

    vec_input = np.array([0.0 for x in range(8 + num_actions)], np.float64)
    func_state = state.time_only_state()
    travel_id_pairs = zip(travel_locations, range(len(travel_locations)))
    location_idx = reduce(lambda x, y: y[1] if y[0] == action.location else x, travel_id_pairs, -1)
    if location_idx < 0: # Should never happen
        raise IndexError

    vec_input[0] = func_state[0]            # Add time of day to state.
    vec_input[1 + func_state[1]] = 1.0      # set to 1 the element corresponding to the current day (skipping the first element).
    vec_input[8 + location_idx] = 1.0       # Set 1 to the element corresponding to the action taken.

    return np.mat(vec_input)

def state_action_reduced_range():
    travel_locations = travel_data.get_location_ids()
    num_actions = len(travel_locations)
    days_in_week = 7

    ranges = []

    # Range for time
    ranges += [(0, 86400)]
     # Actions and day of week take on values from 0 to 1
    ranges += [(0.0, 1.0) for x in range(num_actions + days_in_week)]

    return ranges



def probability_interval(interval, start_time, end_time, stddev, arrival):
    mean = interval.start.second if arrival else interval.end.second
    distribution = scipy.stats.norm(mean, stddev)
    return distribution.cdf(end_time) - distribution.cdf(start_time)

def sample_probability_interval(interval, start_time, end_time, stddev, arrival):
    mean = interval.start.second if arrival else interval.end.second
    min_trunc, max_trunc = (start_time - mean) / stddev, (end_time - mean) / stddev
    return scipy.stats.truncnorm.rvs(min_trunc, max_trunc, loc=mean, scale=stddev)

def transition_func(state, action):
    # Update location and time
    travel_stats = travel_data.stat_travel_time(
                        travel_data.to_pairs_list(
                            travel_data.get_travel_data(),
                            travel_data.get_locations()))
    curr_location = state.location
    next_location = action.location
    travel_time = travel_stats[(curr_location, next_location)]["mean"]
    time_of_day = state.time_of_day + travel_time

    # New request to add:
    new_request_list = state.request_history + [Request(location=next_location,
                                                        time_of_day=time_of_day,
                                                        day_of_week=state.day_of_week)]

    # Dynamics of people moving
    new_present_map = state.person_present_map
    for person in new_present_map:
        schedule = schedule_data.get_schedule_data()[person]

        if new_present_map[person] is None or not new_present_map[person]:
            # Check if person will arrive
            if "schedule" in schedule:
                # Room with fixed schedules
                intervals = util.parse_schedule(schedule["schedule"])

                # Each person's arrival to their office is modeled as a normal distribution centered around their scheduled time of arrival.
                p_intervals = (probability_interval(interval, state.time_of_day, time_of_day, arrival_stddev, True) for interval in intervals)
                for idx, prob in enumerate(p_intervals):
                    if random.random() < prob:
                        new_present_map[person] = int(sample_probability_interval(intervals[idx], state.time_of_day, time_of_day, arrival_stddev, True))
                        break
            elif "intervals-daily" in schedule:
                # Open area
                daily_intervals = schedule["intervals-daily"]
                p_second_arrival = daily_intervals / 24.0 / 60.0 / 60.0
                p_arrival = p_second_arrival * travel_time
                if random.random() < p_arrival:
                    new_present_map[person] = time_of_day
        else:
            # Remove person if appropriate

            # Scheduled offices
            if "schedule" in schedule:
                intervals = util.parse_schedule(schedule["schedule"])

                # Each person's departure from their office is modeled as a normal distribution centered around their scheduled time of departure.
                p_intervals = (probability_interval(interval, state.time_of_day, time_of_day, departure_stddev, False) for interval in intervals)
                for idx, prob in enumerate(p_intervals):
                    if random.random() < prob:
                        new_present_map[person] = None
                        break

            # Non-scheduled offices (mostly open spaces)
            elif "intervals-daily" in schedule:
                interval = util.Interval(
                                util.Time(state.time_of_day, new_present_map[person]),
                                util.Time(state.time_of_day, new_present_map[person] + schedule["duration"]))
                p_interval = probability_interval(interval, state.time_of_day, time_of_day, open_stddev, False)
                if random.random() < p_interval:
                    new_present_map[person] = None

    # Resulting state
    return FullState(location=next_location,
                     time_of_day=time_of_day,
                     day_of_week=state.day_of_week,
                     person_present_map=new_present_map,
                     request_history=new_request_list)

def reward_func(state, action, newState):
    if len(newState.request_history) > len(state.request_history):
        latest_request = newState.request_history[-1]
        if(newState.person_present_map[latest_request.location]):
            loc_schedule_data = schedule_data.get_schedule_data()[latest_request.location]
            p_answer = loc_schedule_data["panswer"] if "panswer" in loc_schedule_data else 1
            return 1 if random.random() < p_answer else 0

    return 0
