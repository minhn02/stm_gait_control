""" Class for controlling a CUI Devices AMT223C-V Absolute Encoder

Datasheet: https://www.cuidevices.com/product/resource/amt22.pdf
Example code for Arduino: https://www.cuidevices.com/product/resource/sample-code/amt22

The "C" in the part number refers to the absolute resolution. C = 12-bit multi-turn.
"""

import time
from contextlib import contextmanager
from typing import Optional

import RPi.GPIO as GPIO
import spidev

CMD_READ = [0x00, 0x00]
CMD_RESET = [0x00, 0x60]
CMD_READ_TURNS = [0x00, 0xA0, 0x00, 0x00]  # Multi-turn encoders only

GPIO.setmode(GPIO.BCM)


def check_parity(msb: int, lsb: int) -> bool:

    # The first two bits of the 16-bit message are the checksums
    odd_checksum = msb >> 7 & 1
    even_checksum = msb >> 6 & 1

    # Odd parity is 1 if there are an even number of 1s in the group and 0 if there are an odd
    # number of 1s. Odd parity is calculated for the odd bits and also for the even bits.
    odd_parity_odd_bits = 1 - (
        (msb >> 5 & 1)
        ^ (msb >> 3 & 1)
        ^ (msb >> 1 & 1)
        ^ (lsb >> 7 & 1)
        ^ (lsb >> 5 & 1)
        ^ (lsb >> 3 & 1)
        ^ (lsb >> 1 & 1)
    )
    odd_parity_even_bits = 1 - (
        (msb >> 4 & 1)
        ^ (msb >> 2 & 1)
        ^ (msb >> 0 & 1)
        ^ (lsb >> 6 & 1)
        ^ (lsb >> 4 & 1)
        ^ (lsb >> 2 & 1)
        ^ (lsb >> 0 & 1)
    )

    return odd_checksum == odd_parity_odd_bits and even_checksum == odd_parity_even_bits


@contextmanager
def gpio_chip_select(gpio_chip_select_bcm: Optional[int]):
    """If a GPIO pin is being used for CS, pull low (meaning selected) while in context."""
    # Manual chip select
    if gpio_chip_select_bcm is not None:
        GPIO.output(gpio_chip_select_bcm, GPIO.LOW)

    yield

    # Manual chip deselect
    if gpio_chip_select_bcm is not None:
        GPIO.output(gpio_chip_select_bcm, GPIO.HIGH)


class Encoder:
    def __init__(
        self,
        chip_select: Optional[int] = None,
        gpio_chip_select_bcm: Optional[int] = None,
        bus: int = 0,
        max_speed_hz: int = 488000,
    ):
        # 488000 Hz was used for Asterix and Obelix; the AMT22 datasheet says it can do a datarate
        # up to 2MHz, but the checksum bits fail at that speed
        if chip_select is None and gpio_chip_select_bcm is None:
            raise ValueError("chip_select and gpio_chip_select_bcm must not both be None")
        self.gpio_chip_select_bcm = gpio_chip_select_bcm
        self.spi = spidev.SpiDev()
        self.spi.open(bus, chip_select if chip_select is not None else 0)
        self.spi.max_speed_hz = max_speed_hz
        self.delay_us = 40
        if self.gpio_chip_select_bcm is not None:
            self.spi.no_cs = True
            GPIO.setup(self.gpio_chip_select_bcm, GPIO.OUT)
            GPIO.output(self.gpio_chip_select_bcm, GPIO.HIGH)

    def get_pos(self) -> float:

        with gpio_chip_select(self.gpio_chip_select_bcm):
            rcv = self.spi.xfer2(CMD_READ, self.spi.max_speed_hz, self.delay_us)

        msb, lsb = rcv

        valid = check_parity(msb, lsb)
        if not valid:
            raise ValueError(f"Parity check failed ({bin((msb << 8) + lsb)})")

        # Concatenate the two bytes, remove the 2 leading checksum bits, shift right by 2 bits
        # because of 12-bit resolution
        pos = (((msb << 8) + lsb) & 0x3FFF) >> 2

        # Convert to degrees
        pos_deg = pos / 2**12 * 360

        return pos_deg

    def reset(self):
        print("RESET")
        with gpio_chip_select(self.gpio_chip_select_bcm):
            self.spi.xfer2(CMD_RESET, self.spi.max_speed_hz, self.delay_us)
        time.sleep(1)  # Allow time for encoder to start back up

    def __del__(self):
        self.spi.close()
        if self.gpio_chip_select_bcm is not None:
            GPIO.cleanup(self.gpio_chip_select_bcm)
