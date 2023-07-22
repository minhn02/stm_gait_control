import stm_state_machine
from datetime import timedelta
import time
from sshkeyboard import listen_keyboard
import threading

state_machine = stm_state_machine.StateMachine(timedelta(microseconds=1))
commands = state_machine.execute(timedelta(microseconds=10000), {})

n_transitions = 5
squirming_period = 11066666666
wheel_walking_period = 23066666666
squirming_transition_times = [(squirming_period/n_transitions)*(i+1) for i in range(n_transitions)]
wheel_walking_transition_times = [(wheel_walking_period/n_transitions)*(i+1) for i in range(n_transitions)]
squirming_transition_index = 0
wheel_walking_transition_index = 0
transitions_started = False

state_start_time = 0
last_state = 0 # 0 is squirming and 1 is wheel walking

control_period = 0.02

print(squirming_transition_times, wheel_walking_transition_times)

def press(key):
    global state_start_time, transitions_started
    if key == 'a':
        state_machine.switchState(1, timedelta(microseconds=10))
        state_start_time = time.time_ns()
        transitions_started = True
        print("transitions started")



thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True))
thread.start()

control_period = 0.1 #seconds

while True:
    start_time = time.time_ns()
    # check for state change
    
    if transitions_started:
        if state_machine.inTransition():
            state_start_time = time.time_ns()
            print("in transition state")
        elif state_machine.getCurrState() == 0:
            if time.time_ns() - state_start_time > squirming_transition_times[squirming_transition_index]:
                state_machine.switchState(1, timedelta(microseconds=time.time_ns()//1000))
                squirming_transition_index += 1
                print("switching to wheel walking, transition index: ", squirming_transition_index)
        elif state_machine.getCurrState() == 1:
            if time.time_ns() - state_start_time > wheel_walking_transition_times[wheel_walking_transition_index]:
                state_machine.switchState(0, timedelta(microseconds=time.time_ns()//1000))
                wheel_walking_transition_index += 1
                print("switching to squirming, transition index: ", wheel_walking_transition_index)

    # execute state machine
    commands = state_machine.execute(timedelta(microseconds=time.time_ns()//1000), {})
    
    # send commands to rover
    # print(commands[stm_state_machine.Joint.STEERING_JOINT_VEL])

    # wait for next control period
    end_time = time.time_ns()
    # print("loop rate: ", 1/((end_time - start_time)/1e9))

    time.sleep(max(0, control_period - (end_time - start_time)/1e9))