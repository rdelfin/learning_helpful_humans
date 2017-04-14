#! /usr/bin/env python
from firebase import firebase

def generate_count_map(locations, measurements):
    count_map = {}
    for a in locations:
        for b in locations:
            if(a != b):
                count_map[(a, b)] = 0
            
    for measure in measurements:
        key = (measure["from"], measure["to"])
        count_map[key] += 1
    return count_map
        
def get_first_min_map(the_map):
    if(len(the_map.keys()) == 0):
        return ""
    
    min_key = the_map.keys()[0]
    for key in the_map:
        if(the_map[key] < the_map[min_key]):
            min_key = key
            
    return min_key

#if __name__ == "__main__":
#    firebase = firebase.FirebaseApplication("https://robotimages-dacc9.firebaseio.com", None)
#    locs = firebase.get('/locations', None)
#    measurements = firebase.get('/locationtravel', None)
#    if not(measurements is None):
#        count_map = generate_count_map(locs, measurements)
    
    