import numpy as np

class GaitSwitcher:
    def __init__(self, gait_indexes: list[int], gait_periods: list[float], num_switches: int):
        self.has_scheduled_time = False
        self.scheduled_time = 0
        self.gait_times = {}
        self.done = False

        for i in range(len(gait_indexes)):
            switching_times = np.linspace(0, gait_periods[i], num_switches)
            self.gait_times[gait_indexes] = switching_times

    def schedule_time(self, curr_state: int, curr_time: int):
        if curr_state in self.gait_times:
            for switching_time in self.gait_times[curr_state]:
                if switching_time > curr_time:
                    self.scheduled_time = switching_time
                    self.gait_times[curr_state].remove(switching_time)
                    break
        
        # check if gait switching times are empty
        all_empty = True
        for switching_times in self.gait_times.values():
            if switching_times != []:
                all_empty = False
        self.done = all_empty
                
    def ready(self, gait_time: int, curr_state: int) -> bool:
        # find closest time in current gait to switch to
        if not self.has_scheduled_time:
            self.schedule_time(curr_state, gait_time)
            self.has_scheduled_time = True
        else:
            if gait_time >= self.scheduled_time:
                return True
        return False
    
    def get_done(self):
        return self.done