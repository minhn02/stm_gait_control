import stm_state_machine
from datetime import timedelta
import time
from sshkeyboard import listen_keyboard
import threading
from gait_switcher import GaitSwitcher

state_machine = stm_state_machine.StateMachine(timedelta(microseconds=1))

control_period = 0.02

# switching logic
n_transitions = 5
squirming_period = 11066666666
wheel_walking_period = 23066666666
curr_gait_index = 1
gait_switcher = GaitSwitcher([1, 2], [squirming_period, wheel_walking_period], n_transitions)

def send_command(command: int):
    global curr_gait_index
    curr_gait_index = command
    state_machine.switchState(command, timedelta(microseconds=time.time_ns()/1000))

def press(key):
    if key == 'a':
        send_command(1)
        gait_switcher.begin_switching()
        print("transitions started")

thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True))
thread.start()

control_period = 0.1 #seconds

gait_names = ["IDLE", "SQUIRM", "WHEEL_WALKING", "TRANSITION", "STARTUP", "INITIAL"]

send_command(5)

while True:
    start_time = time.time_ns()
    
    # check for switching
    gait_time = state_machine.getCurrGaitTime(timedelta(microseconds=(int)(time.time_ns()//1000)))
    if not state_machine.inTransition():
        if gait_switcher.ready(gait_time, curr_gait_index):
            curr_gait_index = 1 if curr_gait_index == 2 else 2
            send_command(curr_gait_index)
            print("switching state to: ", gait_names[curr_gait_index])
            
            if gait_switcher.done:
                print("TRANSITIONS DONE")

    # execute state machine
    commands = state_machine.execute(timedelta(microseconds=time.time_ns()//1000), {})
    print("steering joint velocity", commands[stm_state_machine.Joint.STEERING_JOINT_VEL])

    # send commands to rover
    # print(commands[stm_state_machine.Joint.STEERING_JOINT_VEL])

    # wait for next control period
    end_time = time.time_ns()
    # print("loop rate: ", 1/((end_time - start_time)/1e9))

    time.sleep(max(0, control_period - (end_time - start_time)/1e9))