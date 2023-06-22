import stm_state_machine
from datetime import timedelta
import time
import peripherals.hebis.hebiX5 as hebi
import peripherals.wheels.wheel_control as wheels

#initialize state machine with current time in nanoseconds (have to convert to microseconds for timedelta support)
stm_state_machine.StateMachine(timedelta(microseconds=time.time_ns()//1000))

while True:
    # check for state change

    # execute state machine
    commands = stm_state_machine.execute(timedelta(microseconds=time.time_ns()//1000), {})

    # send commands to rover
    hebi.Hebi.cmd_position()

    wheelSpeeds = [commands[stm_state_machine.Joint.LEFT_FRONT_WHEEL], commands[stm_state_machine.Joint.RIGHT_FRONT_WHEEL],
                   commands[stm_state_machine.Joint.LEFT_REAR_WHEEL], commands[stm_state_machine.Joint.RIGHT_REAR_WHEEL]]
    wheels.WheelControl.set_speeds(wheelSpeeds)