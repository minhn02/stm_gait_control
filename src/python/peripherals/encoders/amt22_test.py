"""Test the functionality of the amt22.py module.

Run on a Raspberry Pi with CUI Devices AMT223C-V Absolute Encoder connected SPI0 with chip select of
0.

Run test functions from the duaxel-proto/src directory with:
    > python -m pytest peripherals/encoders/amt22_test.py

Run main script from the duaxel-proto/src directory with:
    > python -m peripherals.encoders.amt22_test
"""

import time

from peripherals.encoders.amt22 import Encoder, check_parity
from peripherals.encoders.config import ENFR_CS, ENFY_CS, ENRY_CS


def test_get_pos():
    # Requires device to be connected
    encoder = Encoder()
    pos_deg = encoder.get_pos()
    assert 0 <= pos_deg <= 360


def test_reset():
    # Requires device to be connected
    encoder = Encoder()
    encoder.get_pos()
    encoder.reset()
    encoder.get_pos()


def test_check_parity_valid():
    valid = check_parity(0x61, 0xAB)  # Values from datasheet example
    assert valid


def test_check_parity_invalid():
    valid = check_parity(0b01000, 0)
    assert not valid


if __name__ == "__main__":
    # Continuous polling and printing for general dev testing

    encoder_front_roll = Encoder(**ENFR_CS)
    encoder_front_yaw = Encoder(**ENFY_CS)
    encoder_rear_yaw = Encoder(**ENRY_CS)

    while True:
        try:
            fr_pos_deg = encoder_front_roll.get_pos()
        except ValueError as err:
            print(err)
            fr_pos_deg = 0
        try:
            fy_pos_deg = encoder_front_yaw.get_pos()
        except ValueError as err:
            print(err)
            fy_pos_deg = 0
        try:
            ry_pos_deg = encoder_rear_yaw.get_pos()
        except ValueError as err:
            print(err)
            ry_pos_deg = 0

        print(
            f"Front Roll: {fr_pos_deg:.1f}° | Front Yaw: {fy_pos_deg:.1f}° | Rear Yaw: {ry_pos_deg:.1f}°"
        )

        time.sleep(0.5)
