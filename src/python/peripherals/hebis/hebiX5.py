import math
import time
from typing import Optional

import hebi
import numpy as np

EFFORT_MAX = np.PINF
EFFORT_MIN = np.NINF

PITCH_AVG_RATE_DEG_PER_SEC = 10

def lookup_hebis() -> hebi.Lookup:
    """Search for network connected HEBI modules."""
    lookup = hebi.Lookup()
    time.sleep(2)
    print("Modules found on network:")
    for entry in lookup.entrylist:
        print(f"{entry.family} | {entry.name}")
    return lookup


class Hebi:
    # Search for HEBIs when module is loaded
    lookup = lookup_hebis()

    def __init__(self, family_name: str, module_name: str, cmd_lifetime: Optional[float] = None):

        self.module = Hebi.lookup.get_group_from_names([family_name], [module_name])
        if self.module is None:
            raise RuntimeError("Could not find HEBI module")

        if cmd_lifetime is not None:
            self.module.command_lifetime = cmd_lifetime

        self.feedback = hebi.GroupFeedback(self.module.size)
        self.command = hebi.GroupCommand(self.module.size)

    def cmd_position(self, pos_rad: float, vel_rad_per_sec: Optional[float] = None):
        if pos_rad > math.pi * 2 or pos_rad < -math.pi * 2:
            raise ValueError("pos_rad must be between -2*pi and 2*pi")

        self.command.position = pos_rad

        if vel_rad_per_sec is None:
            vel_rad_per_sec = np.nan

        self.command.velocity = vel_rad_per_sec

        res = self.module.send_command(self.command)
        if not res:
            raise RuntimeError("Could not send command to HEBI")

    def disable_torque(self):
        self.command.effort_limit_max = 0.001
        self.command.effort_limit_min = -0.001
        res = self.module.send_command(self.command)
        if not res:
            raise RuntimeError("Could not send command to HEBI")

    def enable_torque(self):
        self.command.effort_limit_max = EFFORT_MAX
        self.command.effort_limit_min = EFFORT_MIN
        res = self.module.send_command(self.command)
        if not res:
            raise RuntimeError("Could not send command to HEBI")

    def get_feedback(self) -> hebi.GroupFeedback:
        if not self.module.send_feedback_request():
            raise RuntimeError("[HEBI] Could not send feedback request")
        telem = self.module.get_next_feedback(reuse_fbk=self.feedback)
        if telem is None:
            raise RuntimeError("[HEBI] Did not receive feedback")

        return telem

    def generate_trajectory(self, current_pos: float, goal_pos: float, num_waypoints: int = 5):
        pos = np.empty((1, num_waypoints))
        vel = np.empty((1, num_waypoints))
        acc = np.empty((1, num_waypoints))

        # First and last waypoints should have 0 vel and acc
        vel[:, 0] = acc[:, 0] = 0.0
        vel[:, -1] = acc[:, -1] = 0.0
        # Values to be calculated are set to NaN
        vel[:, 1:-1] = acc[:, 1:-1] = np.nan

        pos[0, 0] = current_pos
        pos[0, -1] = goal_pos
        pos[:, 1:-1] = np.nan

        duration_s = math.degrees(abs(goal_pos-current_pos))/PITCH_AVG_RATE_DEG_PER_SEC

        time = np.linspace(0.0, duration_s, num_waypoints)

        trajectory = hebi.trajectory.create_trajectory(time, pos, vel, acc)
        return trajectory
