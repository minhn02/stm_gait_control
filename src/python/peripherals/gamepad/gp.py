"""Simple example showing how to get gamepad events.

Used to get the exact names for specific buttons on the controller.
"""

import time

from inputs import devices, get_gamepad


def main():
    """Just print out some event infomation when the gamepad is used."""
    print(devices[0])
    while 1:
        events = get_gamepad()
        for event in events:
            if event.ev_type == "Key" or event.ev_type == "Absolute":
                print(event.ev_type, event.code, event.state)
            # time.sleep(1)


if __name__ == "__main__":
    main()
