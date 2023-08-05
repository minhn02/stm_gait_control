import numpy as np
from typing import List, Tuple

class GaitSwitcher:
    def __init__(self, gait_indexes: List[int], gait_periods: List[float], num_switches: int):
        self.has_scheduled_time = False
        self.scheduled_time = 0
        self.gait_times = {}
        self.begin = False
        self.done = False

        for i in range(len(gait_indexes)):
            switching_times = np.linspace(1e10, gait_periods[i] - 1e10, num_switches).tolist()
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
            for switching_time in self.gait_times[curr_state]:
                if switching_time > curr_time:
                    self.scheduled_time = switching_time
                    self.gait_times[curr_state].remove(switching_time)
                    break
                
    def ready(self, gait_time: int, curr_state: int) -> bool:
        # find closest time in current gait to switch to
        if not self.begin or self.done:
            return False
                        
        if not self.has_scheduled_time:
            self.schedule_time(curr_state, gait_time)
            self.has_scheduled_time = True
        else:
            if gait_time >= self.scheduled_time:
                self.has_scheduled_time = False
                return True
        return False
    
    def begin_switching(self):
        self.begin = True