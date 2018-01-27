class Time:
    def __init__(self, day, second):
        self.day = day
        self.second = second
    
    def __repr__(self):
        day_map = {0: "Monday", 1: "Tuesday", 2: "Wednesday", 3: "Thursday", 4: "Friday", 5: "Saturday", 6: "Sunday"}
        day_name = day_map[self.day] if self.day in day_map else "UNKNOWN DAY " + str(self.day)
        hours = self.second // 3600
        minutes = (self.second % 3600) // 60
        secs = self.second % 60

        return "%02d:%02d:%02d on %s" % (hours, minutes, secs, day_name)

def parse_schedule(schedule):
    return [Interval(Time(item["start"]["day"], item["start"]["time"]*60),
                     Time(item["end"]["day"],   item["end"]["time"]*60))
            for item in schedule]

class Interval:
    def __init__(self, start, end):
        # This is not usual, but needed for this problem
        assert start.day == end.day

        if start.second < end.second:
            self.start = start
            self.end = end
        else:
            self.start = end
            self.end = start
    
    def __repr__(self):
        return "[%s,%s]" % (repr(self.start), repr(self.end))

    def contains(self, time):
        if time.day != self.start.day:
            return False

        return time.second >= self.start.second and time.second <= self.start.end

    def dist_start(self, time):
        second_diff = time.second - self.start.second
        #day_diff = time.day - self.start.day

        #if second_diff < 0:
        #    day_diff -= 1
        #    second_diff = self.start.second - time.second
        #
        #return Time(day_diff, second_diff)
        return second_diff
