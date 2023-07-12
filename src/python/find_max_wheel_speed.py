from peripherals.wheels.wheel_control import WheelControl

wheels = WheelControl( [ 1, 2, 3, 4 ], [ False, True, False, True ] )

print("Wheel max speed:", wheels.max_speed())