import time

from peripherals.leds.config import BUTTON_PIN_BCM, STOP_LED_PIN_BCM
from peripherals.leds.indicator import LED, Button


def test_stop_led():
    stop_led = LED(STOP_LED_PIN_BCM)
    stop_led.on()
    time.sleep(2)
    stop_led.off()


def test_button():
    button = Button(BUTTON_PIN_BCM)
    while True:
        if button.event_detected():
            print("Event Detected")
            break


if __name__ == "__main__":
    test_stop_led()
