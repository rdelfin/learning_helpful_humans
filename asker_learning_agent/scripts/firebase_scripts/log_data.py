import pyrebase
import firebase_scripts.__config__ as config
import datetime

class RunLogger:
    def __init__(self):
        self.firebase_app = pyrebase.initialize_app(config.firebase_data)
        self.db = self.firebase_app.database()
        self.run_id = datetime.datetime.now().format("%Y%m%dT%H%M%S")


    def record_travel_time(self, from_loc, to_loc, time, timestamp):
        travel_times = self.db.child('runs').child(self.run_id).child('travel').get().val()
        if travel_times is None:
            travel_times = []

        travel_times += [{'from': from_loc, 'to': to_loc, 'time': time, 'timestamp': timestamp}]
        self.db.child('runs').child(self.run_id).child('travel').set(travel_times)

    def record_response(self, location, success, timestamp):
        responses = self.db.child('runs').child(self.run_id).child('travel').get().val()
        if responses is None:
            responses = []

        responses += [{'location': location, 'success': success, 'timestamp': timestamp}]
        self.db.child('runs').child(self.run_id).child('travel').set(responses)

    def record_action(self, location, action, timestamp):
        actions = self.db.child('runs').child(self.run_id).child('actions').get().val()
        if actions is None:
            actions = []

        actions += [{'from_location': location, 'action': action, 'timestamp': timestamp}]
        self.db.child('runs').child(self.run_id).child('actions').set(actions)
