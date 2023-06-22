import stm_state_machine
from datetime import timedelta

state_machine = stm_state_machine.StateMachine(timedelta(microseconds=1))
state_machine.switchState(1, timedelta(microseconds=10))
commands = state_machine.execute(timedelta(microseconds=10000), {})