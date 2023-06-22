"""Test the functionality of the hebiX5.py module.

Run on a Raspberry Pi with at least one HebiX5 connected to Ethernet with appropriate networking
configuration.

Run test functions from the duaxel-proto/src directory with:
    > python -m pytest peripherals/hebis/hebiX5_test.py 

Run main script from the duaxel-proto/src directory with:
    > python -m peripherals.hebis.hebiX5_test
"""

import math
import sys
import time

from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_FRONT_NAME, HEBI_REAR_NAME
from peripherals.hebis.hebiX5 import Hebi, lookup_hebis

DEFAULT_POS_GOAL_DEG = 0


def test_lookup_hebis():
    lookup = lookup_hebis()
    assert len(list(lookup.entrylist)) > 0


def test_trajectory(module: Hebi, pos_goal_deg: float):
    fbk = module.get_feedback()
    current_pos = fbk.position[0]

    trajectory = module.generate_trajectory(current_pos, math.radians(pos_goal_deg))

    duration = trajectory.duration
    t = 0.0
    period = 0.03
    while t < duration:
        pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t)
        module.cmd_position(pos_cmd, vel_cmd)
        t = t + period
        time.sleep(period)


def test_double_trajectory(front_module: Hebi, rear_module: Hebi, pos_goal_deg: float):
    front_fbk = front_module.get_feedback()
    rear_fbk = rear_module.get_feedback()

    current_front_pos = front_fbk.position[0]
    current_rear_pos = rear_fbk.position[0]

    front_trajectory = front_module.generate_trajectory(
        current_front_pos, math.radians(pos_goal_deg)
    )
    rear_trajectory = rear_module.generate_trajectory(current_rear_pos, math.radians(pos_goal_deg))

    front_duration = front_trajectory.duration
    rear_duration = rear_trajectory.duration
    duration = max(front_duration, rear_duration)

    t = 0.0
    period = 0.03

    while t < duration:
        for trajectory, module in (
            (front_trajectory, front_module),
            (rear_trajectory, rear_module),
        ):
            pos_cmd, vel_cmd, acc_cmd = trajectory.get_state(t - (duration - trajectory.duration))
            module.cmd_position(pos_cmd, vel_cmd)
        t = t + period
        time.sleep(period)


if __name__ == "__main__":
    pitch_motor_front = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_FRONT_NAME)
    pitch_motor_rear = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_REAR_NAME)

    duration_s = 5
    control_period_s = 0.02

    front_fbk = pitch_motor_front.get_feedback()
    rear_fbk = pitch_motor_rear.get_feedback()

    front_pos = front_fbk.position[0]
    rear_pos = rear_fbk.position[0]
    print(f"Front Pos = {front_pos:.2f} | Rear Pos = {rear_pos: .2f}")

    if len(sys.argv) == 2:
        pos_goal_deg = float(sys.argv[1])
    else:
        pos_goal_deg = DEFAULT_POS_GOAL_DEG

    print("Test Trajectory")
    # test_trajectory(pitch_motor_front, pos_goal_deg)

    test_double_trajectory(pitch_motor_front, pitch_motor_rear, pos_goal_deg)

    # t_start = time.time()
    # while time.time() < t_start + duration_s:
    #     t_start_loop_ns = time.time_ns()
    #     pitch_motor_front.cmd_position(math.pi / 2, 1)
    #     pitch_motor_rear.cmd_position(math.pi / 2, 1)
    #     elapsed_time_s = (time.time_ns() - t_start_loop_ns) * 0.000000001
    #     time.sleep(max(0, control_period_s - elapsed_time_s))

    # time.sleep(3)

    # t_start = time.time()
    # while time.time() < t_start + duration_s:
    #     t_start_loop_ns = time.time_ns()
    #     pitch_motor_front.cmd_position(0, 1)
    #     pitch_motor_rear.cmd_position(0, 1)
    #     elapsed_time_s = (time.time_ns() - t_start_loop_ns) * 0.000000001
    #     time.sleep(max(0, control_period_s - elapsed_time_s))

    # time.sleep(3)
