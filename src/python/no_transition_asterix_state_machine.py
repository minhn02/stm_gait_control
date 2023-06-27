import stm_state_machine
from datetime import timedelta
import time
from peripherals.hebis.config import HEBI_FAMILY_NAME, HEBI_STEER_NAME, HEBI_BOGIE_NAME
import peripherals.hebis.hebiX5 as hebi
from peripherals.wheels.wheel_control import WheelControl
import curses
import threading
from trapezoid_traj import TrapezTraj
from telem.record_telemetry import *

DEG_TO_RAD = 3.1415/180.0
control_period = 0.02 #seconds
steering_limit = 40*DEG_TO_RAD
bogie_limit = 15*DEG_TO_RAD
transient_duration = 0.5
transition_started = False
transition_generated = False
steering_joint_traj = TrapezTraj(0, 0, 0)
bogie_joint_traj = TrapezTraj(0, 0, 0)
state_to_switch_to = 0

#initialize state machine with current time in nanoseconds (have to convert to microseconds for timedelta support)
state_machine = stm_state_machine.StateMachine(timedelta(microseconds=time.time_ns()//1000))
state_machine.switchState(4) #disables transitions

#initialize motors
steering_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_STEER_NAME)
bogie_motor = hebi.Hebi(family_name=HEBI_FAMILY_NAME, module_name=HEBI_BOGIE_NAME)
wheels = WheelControl([1, 2, 3, 4], [True, False, True, False]) #initialization taken from Arthur's code

filename = "test.csv"
setup_log_file(filename)

def control_loop():
    while True:
        start_time = time.time_ns()    
        
        #log telem
        write_telemetry(filename, start_time, steering_motor, bogie_motor, wheels)

        if transition_started:
            # stop wheels
            wheels.set_speeds([0, 0, 0, 0])

            # generate trapezoidal trajectory back to 0 positions for steering and bogie joints
            if transition_generated:
                if steering_joint_traj.completed() and bogie_joint_traj.completed():
                    transition_started, transition_generated = False, False
                    state_machine.switchState(state_to_switch_to, timedelta(microseconds=time.time_ns()//1000))
                if not steering_joint_traj.completed():
                    desired_steering_pos, desired_steering_vel = steering_joint_traj(time.time())
                    steering_motor.cmd_position(desired_steering_pos, desired_steering_vel)
                if not bogie_joint_traj.completed():
                    desired_bogie_pos, desired_bogie_vel = bogie_joint_traj(time.time())
                    bogie_motor.cmd_position(desired_bogie_pos, desired_bogie_vel)
            else:
                curr_steering_pos = steering_motor.get_feedback().position()
                curr_bogie_pos = bogie_motor.get_feedback().position()
                steering_direction = 1 if curr_steering_pos < 0 else -1
                bogie_direction = 1 if curr_bogie_pos < 0 else -1
                steering_joint_traj = TrapezTraj(-steering_limit, steering_limit, steering_direction * 15*DEG_TO_RAD, transient_duration, t0=time.time())
                bogie_joint_traj = TrapezTraj(-bogie_limit, bogie_limit, bogie_direction * 10*DEG_TO_RAD, transient_duration, t0=time.time())
                transition_generated = True
        else:
            # execute state machine
            commands = state_machine.execute(timedelta(microseconds=time.time_ns()//1000), {})

            # send commands to rover
            steering_motor.cmd_position(commands[stm_state_machine.Joint.STEERING_JOINT], commands[stm_state_machine.Joint.STEERING_JOINT_VEL])
            bogie_motor.cmd_position(commands[stm_state_machine.Joint.BOGIE_JOINT], commands[stm_state_machine.Joint.BOGIE_JOINT_VEL])

            wheelSpeeds = [commands[stm_state_machine.Joint.FRONT_LEFT_WHEEL], commands[stm_state_machine.Joint.FRONT_RIGHT_WHEEL],
                        commands[stm_state_machine.Joint.BACK_LEFT_WHEEL], commands[stm_state_machine.Joint.BACK_RIGHT_WHEEL]]
            wheels.set_speeds(wheelSpeeds)

        # wait for next control period
        end_time = time.time_ns()
        time.sleep(max(0, control_period - (end_time - start_time)/1e9))


#intialize keyboard control
try:
    screen = curses.initscr()
    curses.noecho()
    screen.keypad(True)
    curses.cbreak()
    curses.curs_set(False)
    screen.refresh()

    control_thread = threading.Thread(target=control_loop)
    control_thread.start()

    while True:
        try:
            c = screen.getkey()
        except curses.error as what:
            if str(what) != "no input":
                screen.addstr(3, 0, "Curses error: %s" % str(what))
                screen.clrtoeol()
            c = "None"

        if c == " ":
            transition_started = True
            state_to_switch_to = 0
        elif c == "q":
            transition_started = True
            state_to_switch_to = 0
        elif c == "w":
            transition_started = True
            state_to_switch_to = 1
        elif c == "e":
            transition_started = True
            state_to_switch_to = 2

except KeyboardInterrupt:
    pass

finally:
    screen.addstr(3, 0, "Waiting for the control loop to terminate...")
    screen.clrtoeol()
    screen.refresh()
    control_thread.join()

    curses.endwin()    