import stm_state_machine
from datetime import timedelta
import time

state_machine = stm_state_machine.StateMachine(timedelta(microseconds=1))
state_machine.switchState(2, timedelta(microseconds=10))
commands = state_machine.execute(timedelta(microseconds=10000), {})

control_period = 0.02 #seconds

while True:
    start_time = time.time_ns()
    # check for state change

    # execute state machine
    commands = state_machine.execute(timedelta(microseconds=time.time_ns()//1000), {})
    
    # send commands to rover
    # print(commands)

    # wait for next control period
    end_time = time.time_ns()
    print("loop rate: ", 1/((end_time - start_time)/1e9))

    time.sleep(max(0, control_period - (end_time - start_time)/1e9))