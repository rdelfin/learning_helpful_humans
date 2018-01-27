import pyrebase
import firebase_scripts.__config__ as config
from firebase_scripts import travel_data

def get_schedule_data():
    firebase_app = pyrebase.initialize_app(config.firebase_data)
    db = firebase_app.database()
    schedule_list = db.child("scheduledata").get().val()

    return schedule_list
