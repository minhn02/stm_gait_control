import sys
import time

from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_STEER_NAME, HEBI_BOGIE_NAME
from peripherals.hebis.hebiX5 import Hebi

steering_motor = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_STEER_NAME)
bogie_motor = Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_BOGIE_NAME)

torque_enabled = False
for _ in range(10):
    try:
        steering_motor.enable_torque()
        bogie_motor.enable_torque()
        torque_enabled = True
        break
    except RuntimeError as e:
        print(e, file=sys.stderr)
        time.sleep(0.1)

if torque_enabled:
    print("=== HEBI effort limits restored for operations ===")
