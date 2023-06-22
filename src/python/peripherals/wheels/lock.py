import os
import sys
import time

from peripherals.wheels.wheel_control import WheelControl

param = os.sched_param(10)
os.sched_setscheduler(0, os.SCHED_RR, param)

dxl_ids = [1, 2, 3, 4]

wheels = WheelControl(dxl_ids)

wheel_stopped = False
for _ in range(10):
    try:
        wheels.set_speeds(0)
        wheels.enable()
        wheel_stopped = True
        break
    except RuntimeError as e:
        print(e, file=sys.stderr)
        time.sleep(0.1)

if wheel_stopped:
    print("Wheel stopped and torque enabled")
