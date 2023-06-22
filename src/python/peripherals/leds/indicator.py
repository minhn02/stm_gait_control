import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)


class LED:
    def __init__(self, bcm_pin: int):
        self.bcm_pin = bcm_pin
        GPIO.setup(self.bcm_pin, GPIO.OUT)
        GPIO.output(self.bcm_pin, GPIO.LOW)

    def on(self):
        GPIO.output(self.bcm_pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.bcm_pin, GPIO.LOW)

    # Leave this commented for LED to keep its state after program exits
    # def __del__(self):
    #     GPIO.cleanup(self.bcm_pin)


class Button:
    def __init__(self, bcm_pin: int):
        self.bcm_pin = bcm_pin
        GPIO.setup(self.bcm_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # GPIO.add_event_detect(self.bcm_pin, GPIO.BOTH)

    def is_pressed(self) -> bool:
        return not GPIO.input(self.bcm_pin)

    # def event_detected(self):
    #     return GPIO.event_detected(self.bcm_pin)

    def __del__(self):
        GPIO.cleanup(self.bcm_pin)
