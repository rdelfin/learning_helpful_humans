import pyrebase
import firebase_scripts.__config__ as config

def set_weights(weights):
    """
    Sets the weights stored in firebase. 
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    db.child("training").child("").set(weights)


def get_weights():
    """
    Obtains the weights used by the function estimator from firebase. If the
    field does not exist, None is returned.
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    return db.child("training").child("weights").get().val()


def get_num_actions():
    """
    Gets the number of actions run since the start of the training.
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    num_actions = db.child("training").child("action_count").get().val()

    if num_actions is None:
        set_num_actions(0)
        num_actions = 0

    return num_actions


def set_num_actions(num):
    """
    Sets the number of actions run since the start of the training.
    """
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    return db.child("training").child("action_count").set(num)