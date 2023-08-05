import numpy as np
from typing import List, Tuple

class GaitSwitcher:
    def __init__(self, gait_indexes: List[int], gait_periods: List[float], num_switches: int):
        self.has_scheduled_time = False
        self.scheduled_time = 0
        self.gait_times = {}
        self.begin = False
        self.done = False

        self.before = False

        for i in range(len(gait_indexes)):
            switching_times = np.linspace(1e8, gait_periods[i] - 5e8, num_switches).tolist()
            self.gait_times[gait_indexes[i]] = switching_times
        
    def schedule_time(self, curr_state: int, curr_time: int):
        # check if gait switching times are empty
        all_empty = True
        for switching_times in self.gait_times.values():
            if switching_times != []:
                print(len(switching_times), "left")
                all_empty = False
        self.done = all_empty
        if self.done:
            return
    
        if curr_state in self.gait_times:
            switching_times_np = np.array(self.gait_times[curr_state])
            higher_values = switching_times_np[switching_times_np > (curr_time+1e10)]
            if higher_values.size == 0:
                self.scheduled_time = min(self.gait_times[curr_state])
                self.gait_times[curr_state].remove(self.scheduled_time)
                return
            closest_index = np.argmin(np.abs(higher_values - curr_time))
            self.scheduled_time = self.gait_times[curr_state][closest_index]
            self.gait_times[curr_state].pop(closest_index)
                
    def ready(self, gait_time: int, curr_state: int) -> bool:
        # find closest time in current gait to switch to
        if not self.begin or self.done:
            return False
        
                        
        if not self.has_scheduled_time:
            self.schedule_time(curr_state, gait_time)
            self.has_scheduled_time = True
        else:
            if gait_time < self.scheduled_time:
                self.before = True
                return False
            if self.before and gait_time > self.scheduled_time:
                self.before = False
                self.has_scheduled_time = False
                return True
        return False
    
    def begin_switching(self):
        self.begin = True