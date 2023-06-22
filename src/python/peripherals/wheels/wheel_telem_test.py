"""Test wheel telemetry is being read.

Run main script from the duaxel-proto/src directory with:
    > python -m peripherals.wheels.wheel_telem_test
"""

import time

from rich import print

from peripherals.wheels.wheel_control import WheelControl

if __name__ == "__main__":
    wheels = WheelControl([1, 2, 3, 4], [False, True, False, True])

    while True:
        wheels_telem = wheels.get_telemetry()
        print(wheels_telem)
        time.sleep(1)
