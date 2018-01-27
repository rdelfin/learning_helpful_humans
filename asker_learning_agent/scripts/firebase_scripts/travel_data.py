import pyrebase
import firebase_scripts.__config__ as config
import numpy as np

def stat_travel_time(pairs_list):
    """
    Creates a dictionary of stats for every item in pairs_list. Each item
    contains three statistics:
    "mean": The mean value of travel time for successful trip
    "std_dev": The standard deviation of travel time for successful trip
    "p_complete": An estimate of the probability of completing the trip
    It's important to note that only items in pairs_list with more than
    one trip will be considered (otherwise std_dev is either undefined or
    zero).
    """
    time_dict = {pair: [travel["time"] for travel in pairs_list[pair] if travel["completed"]] for pair in pairs_list}
    complete_dict = {pair: [travel["completed"] for travel in pairs_list[pair]] for pair in pairs_list}
    
    result_dict = {pair: { \
                    "mean": np.mean(time_dict[pair]) if len(time_dict[pair]) > 0 else 110, \
                    "p_complete": sum(complete_dict[pair]) / len(complete_dict[pair]) if len(time_dict[pair]) > 0 else 1 \
                   } \
                   for pair in time_dict}

    return result_dict

def to_pairs_list(travel_data, locations):
    """
    Converts the travel data (a list of dicts with from, to, complete and time
    information) to a dictionary of pairs to a list of dictionaries, each one
    containing a time and completed property
    """
    pairs = set((first, second) for first in locations for second in locations)
    pair_data = {pair: [] for pair in pairs}
    for trip in travel_data:
        trip_pair = (trip["from"], trip["to"])
        if trip_pair not in pairs:
            print("WARNING: Pair (%s, %s) not present. Ignoring..." % trip_pair)
            continue
        pair_data[trip_pair] += [{"time": trip["time"], "completed": trip["completed"]}]

    return pair_data

def get_locations():
    """
    Gets a dict of all locations as used by the robots indexed by their id's.
    This includes a type, classifier type, name, and other information specific
    to the type of location.
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    return db.child("locations").get().val()

def get_location_ids():
    """
    Gets a list of all location id's used by the robot.
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    loc_list = db.child("locations").get()
    return set(location.key() for location in loc_list.each())

def get_travel_data():
    """
    Gets a list of every trip the robot has done. Each trip is a dictionary
    with:
    completed: True/False for whether the trip arrived at the destination
    time: Time taken to complete the trip in seconds
    from: ID of the location where the trip began
    to: ID of the location where the trip ended
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    return db.child("locationtravel").get().val()
