import stm_state_machine
from datetime import timedelta
import time
from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_STEER_NAME, HEBI_BOGIE_NAME
import peripherals.hebis.hebiX5 as hebi
from peripherals.wheels.wheel_control import WheelControl
import threading
from telem.record_telemetry import *
from sshkeyboard import listen_keyboard

DEG_TO_RAD = 3.1415/180.0
control_period = 0.02 #seconds
steering_limit = 40*DEG_TO_RAD
bogie_limit = 15*DEG_TO_RAD

#initialize state machine with current time in nanoseconds (have to convert to microseconds for timedelta support)
state_machine = stm_state_machine.StateMachine(timedelta(microseconds=time.time_ns()/1000))

# initialize motors
steering_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_STEER_NAME)
bogie_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_BOGIE_NAME)
wheels = WheelControl([1, 2, 3, 4], [False, True, False, True]) #initialization taken from Arthur's code

filename = "test.csv"
setup_log_file(filename)

def send_command(command):
    state_machine.switchState(command, timedelta(microseconds=time.time_ns()/1000))

def control_loop():
        start_time = time.time_ns()

        # execute state machine
        wheel_telem = wheels.get_telemetry()
        curr_states = {
            stm_state_machine.Joint.STEERING_JOINT: steering_motor.get_feedback().position[0],
            stm_state_machine.Joint.BOGIE_JOINT: bogie_motor.get_feedback().position[0],
            stm_state_machine.Joint.FRONT_LEFT_WHEEL: wheel_telem[0]['velocity'],
            stm_state_machine.Joint.FRONT_RIGHT_WHEEL: wheel_telem[1]['velocity'],
            stm_state_machine.Joint.BACK_LEFT_WHEEL: wheel_telem[2]['velocity'],
            stm_state_machine.Joint.BACK_RIGHT_WHEEL: wheel_telem[3]['velocity']
        }
        
        commands = state_machine.execute(timedelta(microseconds=time.time_ns()/1000), curr_states)

        # send commands to rover
        # steering_motor.cmd_position(commands[stm_state_machine.Joint.STEERING_JOINT], commands[stm_state_machine.Joint.STEERING_JOINT_VEL])
        # bogie_motor.cmd_position(commands[stm_state_machine.Joint.BOGIE_JOINT], commands[stm_state_machine.Joint.BOGIE_JOINT_VEL])

        # wheelSpeeds = [commands[stm_state_machine.Joint.FRONT_LEFT_WHEEL], commands[stm_state_machine.Joint.FRONT_RIGHT_WHEEL],
        #             commands[stm_state_machine.Joint.BACK_LEFT_WHEEL], commands[stm_state_machine.Joint.BACK_RIGHT_WHEEL]]
        # wheels.set_speeds(wheelSpeeds)

        # log telem
        write_telemetry(filename, start_time/1e9, steering_motor, bogie_motor, wheels, commands[stm_state_machine.Joint.TRANSITIONING])

        # wait for next control period
        end_time = time.time_ns()
        print("loop rate: ", 1/((end_time - start_time)/1e9))
        time.sleep(max(0, control_period - (end_time - start_time)/1e9))

# register keyboard callbacks
def press(key):
    if key == 'q':
        send_command(0)
    elif key == 'w':
        send_command(1)
    elif key == 'e':
        send_command(2)
    elif key == 'a':
        send_command(3)
    elif key == 's':
        send_command(4)
    elif key == 'd':
        send_command(5)
    elif key == 'f':
        send_command(6)

listen_thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True))
listen_thread.start()

while True:
    control_loop()