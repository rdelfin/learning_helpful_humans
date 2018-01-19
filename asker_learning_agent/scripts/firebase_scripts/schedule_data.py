import pyrebase
import firebase_scripts.__config__ as config
from firebase_scripts import travel_data
from functools import lru_cache

@lru_cache(maxsize=100)
def get_schedule_data():
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    schedule_list = db.child("scheduledata").get().val()

    return schedule_list
