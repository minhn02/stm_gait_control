"""Test wheel rotations.

Run main script from the duaxel-proto/src directory with:
    > python -m peripherals.wheels.wheel_rotation_test
"""


import sys
import time

import numpy as np

from peripherals.wheels.wheel_control import WheelControl

RAD_TO_DEG = 180 / np.pi
DEG_TO_RAD = np.pi / 180
RAD_TO_REVS = 1 / (2 * np.pi)
REVS_TO_RAD = 2 * np.pi

WHEEL_RADIUS_MM = 223 / 2  # (equivalent radius on hard soil)


if __name__ == "__main__":

    control_period_s = 0.02

    # set_linear_vel = 50.80337547960883 * 2  # mm/s
    set_linear_vel = 50
    set_total_revs = 1
    set_angular_vel = set_linear_vel / WHEEL_RADIUS_MM
    set_duration = set_total_revs * REVS_TO_RAD / set_angular_vel
    print(f"Set duration: {set_duration:.2f}")

    # IDs of the wheel motors in the following order: front left, fornt right, rear left, rear right
    wheels = WheelControl([1, 2, 3, 4], [False, True, False, True])
    wheels.set_velocity_Kp(300)
    wheels.set_velocity_Ki(3000)

    elapsed_time = 0.0
    measured_revs = 0.0
    Ti = 0.0
    t0 = time.time()
    t = 0.0
    prev_t = 0.0
    measured_angular_vel_sum = 0.0
    data_count = 0

    wheels.set_speeds(set_angular_vel)

    try:
        while t < set_duration:
            start_tick = time.time_ns()

            # Get the telemetry from the wheel motors:
            wheels_tele = wheels.get_telemetry()
            time.sleep(0.01)

            t = time.time() - t0
            T = t - prev_t
            prev_t = t

            measured_angular_vel = wheels_tele[0]["velocity"]
            measured_revs += measured_angular_vel * T * RAD_TO_REVS
            expected_revs = set_angular_vel * t * RAD_TO_REVS

            Ti += T

            print(
                f"t: {t:.2f} | Ti: {Ti:.2f} | T: {T:.2f} | Measured ω: {measured_angular_vel:.2f} | "
                f"Measured Revs: {measured_revs:.2f} | Expected Revs: {expected_revs:.2f}"
            )

            measured_angular_vel_sum += measured_angular_vel
            data_count += 1

            # Compute the elapsed time so far and wait the remaining time to complete the period of the
            # control loop:
            elapsed_time = (time.time_ns() - start_tick) * 0.000000001
            time.sleep(max(0, control_period_s - elapsed_time))

    finally:
        mean_measured_revs = measured_angular_vel_sum / data_count * t * RAD_TO_REVS
        print("--------------------------------------")
        print("| Statistics:                         |")
        print("--------------------------------------")
        print(f"| Measured Revs: \t\t{measured_revs:.3f} |")
        print(f"| Mean Measured Revs: \t\t{mean_measured_revs:.3f} | ")
        print(f"| Set Revs/Measured Revs: \t{set_total_revs/measured_revs:.3f} | ")
        print(f"| Set Revs/Mean Measured Revs: \t{set_total_revs/mean_measured_revs:.3f} |")
        print("--------------------------------------")

        for _ in range(3):
            try:
                time.sleep(control_period_s)
                wheels.disable()
                break
            except RuntimeError as e:
                print(e, file=sys.stderr)
                time.sleep(control_period_s)
                time.sleep(control_period_s)
