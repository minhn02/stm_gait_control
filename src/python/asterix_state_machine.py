import stm_state_machine
from datetime import timedelta
import time
from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_STEER_NAME, HEBI_BOGIE_NAME
import peripherals.hebis.hebiX5 as hebi
from peripherals.wheels.wheel_control import WheelControl
import threading
from telem.record_telemetry import *
from sshkeyboard import listen_keyboard, stop_listening
import numpy as np
import sys
import peripherals.wheels.disable as wheel_disable
from gait_switcher import GaitSwitcher

DEG_TO_RAD = 3.1415/180.0
control_period = 0.02 #seconds
steering_limit = 40*DEG_TO_RAD
bogie_limit = 15*DEG_TO_RAD

filename = "8-5-bezierway-heading-3.csv"
setup_log_file(filename)

n_transitions = 5
squirming_period = 11066666666
wheel_walking_period = 20000000000

#initialize state machine with current time in nanoseconds (have to convert to microseconds for timedelta support)
state_machine = stm_state_machine.StateMachine(timedelta(microseconds=time.time_ns()/1000))

# initialize motors
steering_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_STEER_NAME)
bogie_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_BOGIE_NAME)
wheels = WheelControl([1, 2, 3, 4], [False, True, False, True]) #initialization taken from Arthur's code

Lx = 200
Ly = 200
Ly_Lx = Ly/Lx
WHEEL_RADIUS = 100
MAX_WHEEL_SPEED = wheels.max_speed

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

gait_names = ["IDLE", "SQUIRM", "WHEEL_WALKING", "TRANSITION", "STARTUP", "INITIAL"]
transition_names = ["NAIVE", "BEZIER", "LINEAR_WAYPOINT", "BEZIER_WAYPOINT"]
transition_index = 0

# switching logic
curr_gait_index = 1
gait_switcher = GaitSwitcher([1, 2], [squirming_period, wheel_walking_period], n_transitions)

# bogie_disable_buffer
# need buffer because there's one timestep where the rover is transitioning to squirming and disables the bogie torque
buffer = False

def send_command(command: int):
    global curr_gait_index, transition_index
    if command < 3:
        curr_gait_index = command
    else:
        transition_index = command - 3
    state_machine.switchState(command, timedelta(microseconds=time.time_ns()/1000))

def control_loop():
    global buffer
    start_time = time.time_ns()

    gait_time = state_machine.getCurrGaitTime(timedelta(microseconds=(int)(time.time_ns()//1000)))
    if not state_machine.inTransition():
        if gait_switcher.ready(gait_time, curr_gait_index):
            gait_index = 1 if curr_gait_index == 2 else 2
            send_command(gait_index)
            print("switching state to: ", gait_names[curr_gait_index])
            
        if gait_switcher.done:
            print("TRANSITIONS DONE")

    # set bogie joint to 0 effort when squirming
    if state_machine.getCurrState() == 1 and not state_machine.inTransition():
        if buffer:
            bogie_motor.disable_torque()
        else:
            buffer = True
    else:
         buffer = False
         bogie_motor.enable_torque()

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
    commanded_steer_position = np.clip(commands[stm_state_machine.Joint.STEERING_JOINT], -steering_limit, steering_limit)
    steering_motor.cmd_position(commanded_steer_position, commands[stm_state_machine.Joint.STEERING_JOINT_VEL])
    commanded_bogie_position = np.clip(commands[stm_state_machine.Joint.BOGIE_JOINT], -bogie_limit, bogie_limit)
    bogie_motor.cmd_position(commanded_bogie_position, commands[stm_state_machine.Joint.BOGIE_JOINT_VEL])

    # wheelSpeeds = [commands[stm_state_machine.Joint.FRONT_LEFT_WHEEL], commands[stm_state_machine.Joint.FRONT_RIGHT_WHEEL],
    #             commands[stm_state_machine.Joint.BACK_LEFT_WHEEL], commands[stm_state_machine.Joint.BACK_RIGHT_WHEEL]]
    wheelSpeeds = wheel_speed_synchronization(commands[stm_state_machine.Joint.STEERING_JOINT], commands[stm_state_machine.Joint.STEERING_JOINT_VEL], forward_crawling=True)
    wheels.set_speeds(wheelSpeeds)

    # log telem
    gait_or_transition_name = gait_names[state_machine.getCurrState()] if not state_machine.inTransition() else transition_names[transition_index]
    write_telemetry(filename, start_time/1e9, steering_motor, bogie_motor, wheels, commands[stm_state_machine.Joint.TRANSITIONING], gait_or_transition_name)

    # wait for next control period
    end_time = time.time_ns()
    # print("loop rate: ", 1/((end_time - start_time)/1e9))
    time.sleep(max(0, control_period - (end_time - start_time)/1e9))

# register keyboard callbacks
def press(key):
    if key == 'q':
        print("switching state to idle")
        send_command(0)
    elif key == 'w':
        print("switching state to squirming")
        send_command(1)
    elif key == 'e':
        print("switching state to wheel-walking")
        send_command(2)
    elif key == 'a':
        print("switching transition to naive")
        send_command(3)
    elif key == 's':
        print("switching transition to bezier")
        send_command(4)
    elif key == 'd':
        print("switching transition to linear waypoint")
        send_command(5)
    elif key == 'f':
        print("switching transition to bezier waypoint")
        send_command(6)
    elif key == 'z':
        send_command(1)
        gait_switcher.begin_switching()
        print("transitions started")

listen_thread = threading.Thread(target=lambda: listen_keyboard(on_press=press, sequential=True, until='p'))
listen_thread.start()

while True:
    try:
        control_loop()
    except KeyboardInterrupt:
         print("////// Terminating Program //////")
         wheel_disable.disable()
         listen_thread.join()
         stop_listening()
         break