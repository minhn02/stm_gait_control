import sys
import time

from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_FRONT_NAME, HEBI_REAR_NAME
from peripherals.hebis.hebiX5 import Hebi

pitch_motor_front = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_FRONT_NAME)
pitch_motor_rear = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_REAR_NAME)

torque_enabled = False
for _ in range(10):
    try:
        pitch_motor_front.enable_torque()
        pitch_motor_rear.enable_torque()
        torque_enabled = True
        break
    except RuntimeError as e:
        print(e, file=sys.stderr)
        time.sleep(0.1)

if torque_enabled:
    print("=== HEBI effort limits restored for operations ===")
