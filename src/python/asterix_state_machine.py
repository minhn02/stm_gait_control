import stm_state_machine
from datetime import timedelta
import time
from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_STEER_NAME, HEBI_BOGIE_NAME
import peripherals.hebis.hebiX5 as hebi
from peripherals.wheels.wheel_control import WheelControl
import threading
from telem.record_telemetry import *
from sshkeyboard import listen_keyboard
import numpy as np
import sys
import peripherals.wheels.disable as wheel_disable

DEG_TO_RAD = 3.1415/180.0
control_period = 0.02 #seconds
steering_limit = 40*DEG_TO_RAD
bogie_limit = 15*DEG_TO_RAD

n_transitions = 5
squirming_period = 11066666666
wheel_walking_period = 23066666666
squirming_transition_times = [(squirming_period/n_transitions)*(i+1) for i in range(n_transitions)]
wheel_walking_transition_times = [(wheel_walking_period/n_transitions)*(i+1) for i in range(n_transitions)]
squirming_transition_index = 0
wheel_walking_transition_index = 0
transitions_started = False
state_start_time = 0

#initialize state machine with current time in nanoseconds (have to convert to microseconds for timedelta support)
state_machine = stm_state_machine.StateMachine(timedelta(microseconds=time.time_ns()/1000))

# initialize motors
steering_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_STEER_NAME)
bogie_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_BOGIE_NAME)
wheels = WheelControl([1, 2, 3, 4], [False, True, False, True]) #initialization taken from Arthur's code

filename = "7-21-everything-testing.csv"
setup_log_file(filename)

Lx = 200
Ly = 200
Ly_Lx = Ly/Lx
WHEEL_RADIUS = 100
MAX_WHEEL_SPEED = wheels.max_speed

gait_names = ["IDLE", "SQUIRM", "WHEEL_WALKING", "INITIAL", "NAIVE", "BEZIER", "BEZIER_WAYPOINT", "LINEAR_WAYPOINT"]
curr_gait_index = -1

def send_command(command: int):
    global curr_gait_index
    curr_gait_index = command
    state_machine.switchState(command, timedelta(microseconds=time.time_ns()/1000))

def wheel_speed_synchronization( steering_joint_angle, steering_joint_rate, forward_crawling=True ) :
	b_2 = steering_joint_angle*0.5
	db_2dt = steering_joint_rate*0.5

	vs = np.array( [ ( -1 if i//2 else 1 )*( -Lx*np.tan( b_2 ) + ( -1 if i%2 else 1 )*Ly )*db_2dt for i in range( 4 ) ] )
	cd = np.array( [ 1 + ( -1 if i%2 else 1 )*Ly_Lx*np.tan( b_2 ) for i in range( 4 ) ] )

	if forward_crawling :
		# Set the robot speed so that no wheel is going backward:
		rover_speed = max( -vs/cd )
	else :
		rover_speed = 0

	# Reduce the robot speed if needed according to the wheel speed limit:
	max_rover_speed_per_wheel = ( MAX_WHEEL_SPEED*WHEEL_RADIUS - vs )/cd
	rover_speed_capped = min( rover_speed, min(  max_rover_speed_per_wheel ) )

	# Display which wheels were about to exceeded their maximum velocity before capping the rover speed:
	if rover_speed_capped != rover_speed :
		wheels_at_issue = np.where( max_rover_speed_per_wheel < rover_speed )[0] + 1
		print( '\u26A0 Target exceeds max speed for wheel(s) %s (rover speed is capped to %+.f mm/s instead of %+.f mm/s)'
		% ( str( wheels_at_issue ), rover_speed_capped, rover_speed ), file=sys.stderr )

	# Compute the resulting speed for each wheel:
	Wd = ( rover_speed_capped*cd + vs )/WHEEL_RADIUS

	return Wd

last_transition_time = 0
last_gait_command = 1

def control_loop():
    global state_start_time, squirming_transition_index, wheel_walking_transition_index
    start_time = time.time_ns()

    if transitions_started:
        if state_machine.inTransition():
            state_start_time = time.time_ns()
            print("in transition state")
        elif state_machine.getCurrState() == 1:
            if time.time_ns() - state_start_time > squirming_transition_times[squirming_transition_index]:
                state_machine.switchState(2, timedelta(microseconds=time.time_ns()//1000))
                squirming_transition_index = (squirming_transition_index + 1) % n_transitions
                print("switching to wheel walking, transition index: ", squirming_transition_index)
        elif state_machine.getCurrState() == 2:
            if time.time_ns() - state_start_time > wheel_walking_transition_times[wheel_walking_transition_index]:
                state_machine.switchState(1, timedelta(microseconds=time.time_ns()//1000))
                wheel_walking_transition_index = (wheel_walking_transition_index + 1) % n_transitions
                print("switching to squirming, transition index: ", wheel_walking_transition_index)

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
    steering_motor.cmd_position(commands[stm_state_machine.Joint.STEERING_JOINT], commands[stm_state_machine.Joint.STEERING_JOINT_VEL])
    bogie_motor.cmd_position(commands[stm_state_machine.Joint.BOGIE_JOINT], commands[stm_state_machine.Joint.BOGIE_JOINT_VEL])

    # wheelSpeeds = [commands[stm_state_machine.Joint.FRONT_LEFT_WHEEL], commands[stm_state_machine.Joint.FRONT_RIGHT_WHEEL],
    #             commands[stm_state_machine.Joint.BACK_LEFT_WHEEL], commands[stm_state_machine.Joint.BACK_RIGHT_WHEEL]]
    wheelSpeeds = wheel_speed_synchronization(commands[stm_state_machine.Joint.STEERING_JOINT], commands[stm_state_machine.Joint.STEERING_JOINT_VEL], forward_crawling=True)
    wheels.set_speeds(wheelSpeeds)

    # log telem
    write_telemetry(filename, start_time/1e9, steering_motor, bogie_motor, wheels, commands[stm_state_machine.Joint.TRANSITIONING], gait_names[curr_gait_index])

    # wait for next control period
    end_time = time.time_ns()
    # print("loop rate: ", 1/((end_time - start_time)/1e9))
    time.sleep(max(0, control_period - (end_time - start_time)/1e9))

# register keyboard callbacks
def press(key):
    global state_start_time, transitions_started
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
    elif key == 'z':
        send_command(1)
        transitions_started = True
        state_start_time = time.time_ns()
        print("transitions started")

listen_thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True))
listen_thread.start()

while True:
    try:
        control_loop()
    except KeyboardInterrupt:
         print("////// Terminating Program //////")
         wheel_disable.disable()
         listen_thread.join()
         break