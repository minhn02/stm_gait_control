import collections
import os

import dynamixel_sdk as dxl

# Give realtime priority to the process:
# param = os.sched_param( os.sched_get_priority_max( os.SCHED_RR ) )
# Seem to interfere with serial port
param = os.sched_param(10)
os.sched_setscheduler(0, os.SCHED_RR, param)
# If not permitted, add the line "username - rtprio 99" in /etc/security/limits.conf and reboot


class WheelControl:
    """
    A class to control Dynamixel motors XM540-150 in velocity.

    Parameters
    ----------
    motor_ids : a list of ints
            ID numbers of each Dynamixel motor to use.
    reverse_directions : a list of bools, optional, default: None
            Whether or not the positive direction of rotation of each motor need to be reversed.
    max_speed : float, optional, default: 5.54
            Maximum speed (in rad/s) to be set for all the motors.

    """

    # Protocol:
    protocol_version = 2
    comm_success = 0
    comm_tx_fail = -1001

    # Control table addresses:
    addr_operating_mode = 11
    addr_torque_enable = 64

    addr_velocity_limit = 44
    addr_velocity_I_gain = 76
    addr_velocity_P_gain = 78
    addr_goal_velocity = 104
    len_goal_velocity = 4

    addr_present_current = 126
    len_present_current = 2
    addr_present_velocity = 128
    len_present_velocity = 4
    addr_present_position = 132
    len_present_position = 4
    addr_present_voltage = 144
    len_present_voltage = 2
    addr_present_temperature = 146
    len_present_temperature = 1
    addr_present_pwm = 124
    len_present_pwm = 2

    addr_telemetry_block = 124
    len_telemetry_block = 23

    # Conversion coefficient:
    speed_to_bytes = 41.69998508957956  # 1/( 0.229*2*pi/60 )

    def _error_check(self, header=""):
        if self._comm_result != WheelControl.comm_success:
            raise RuntimeError(
                "%s%s" % (header, self._packetHandler.getTxRxResult(self._comm_result))
            )
        elif self._comm_error != 0:
            raise RuntimeError(
                "%s%s" % (header, self._packetHandler.getRxPacketError(self._comm_error))
            )

    def set_operating_mode(self, mode):
        """Set the operating mode for all the motors"""
        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write1ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_operating_mode, mode
            )
            self._error_check(
                "[DXL] Failed to set the operating mode of motor %d to %d: " % (motor_id, mode)
            )

    def get_operating_modes(self):
        """Get the current operating mode of each motor"""
        modes = []
        for motor_id in self._motor_ids:
            mode, self._comm_result, self._comm_error = self._packetHandler.read1ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_operating_mode
            )
            self._error_check("[DXL] Failed to get the operating mode of motor %d: " % motor_id)
            modes.append(mode)
        return modes

    def get_max_speeds(self):
        """Get the current maximum speed (in rad/s) of each motor"""
        speeds = []
        for motor_id in self._motor_ids:
            speed, self._comm_result, self._comm_error = self._packetHandler.read4ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_velocity_limit
            )
            self._error_check("[DXL] Failed to set the maximum speed of motor %d: " % motor_id)
            speeds.append(speed / WheelControl.speed_to_bytes)
        return speeds

    @property
    def max_speed(self):
        """Return the maximum speed (in rad/s) set for all the motors"""
        return self._max_speed / WheelControl.speed_to_bytes

    @max_speed.setter
    def max_speed(self, max_speed):
        """Set the maximum speed (in rad/s) for all the motors"""

        self._max_speed = int(max_speed * WheelControl.speed_to_bytes)

        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write4ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_velocity_limit, self._max_speed
            )
            self._error_check(
                "[DXL] Failed to set the maximum speed of motor %d to %d: "
                % (motor_id, self._max_speed)
            )

    def set_velocity_Kp(self, Kp=100):
        """Set the proportional gain of the velocity feedback controller"""
        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write2ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_velocity_P_gain, Kp
            )
            self._error_check(
                "[DXL] Failed to set the velocity proportional gain of motor %d to %d: "
                % (motor_id, Kp)
            )

    def set_velocity_Ki(self, Ki=1920):
        """Set the integral gain of the velocity feedback controller"""
        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write2ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_velocity_I_gain, Ki
            )
            self._error_check(
                "[DXL] Failed to set the velocity integral gain of motor %d to %d: "
                % (motor_id, Ki)
            )

    def enable(self):
        """Enable wheel torque"""
        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write1ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_torque_enable, 1
            )
            self._error_check("[DXL] Failed to enable motor %d: " % motor_id)

    def disable(self):
        """Disable wheel torque"""
        for motor_id in self._motor_ids:
            self._comm_result, self._comm_error = self._packetHandler.write1ByteTxRx(
                self._portHandler, motor_id, WheelControl.addr_torque_enable, 0
            )
            self._error_check("[DXL] Failed to disable motor %d: " % motor_id)

    def __init__(
        self,
        motor_ids,
        reverse_directions=None,
        device="/dev/ttyUSB0",
        baudrate=1000000,
        max_speed=5.52,
    ):
        self._comm_result = WheelControl.comm_tx_fail

        if not all([isinstance(motor_id, int) for motor_id in motor_ids]):
            raise TypeError("Argument 'motor_ids' should be a list of integers")
        self._motor_ids = motor_ids

        if reverse_directions is not None:
            self.set_directions(reverse_directions)
        else:
            self._motor_directions = [1] * len(self._motor_ids)

        self._portHandler = dxl.PortHandler(device)
        self._packetHandler = dxl.PacketHandler(WheelControl.protocol_version)

        if not self._portHandler.openPort():
            raise RuntimeError("[DXL] Failed to open port %s" % device)

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError("[DXL] Failed to set baurate to %d" % baudrate)

        # Declare sync write to set the desired speeds:
        self._groupSyncWrite = dxl.GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            WheelControl.addr_goal_velocity,
            WheelControl.len_goal_velocity,
        )

        # Declare sync read to fetch telemetry in a block:
        self._groupSyncRead = dxl.GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            WheelControl.addr_telemetry_block,
            WheelControl.len_telemetry_block,
        )
        for motor_id in self._motor_ids:
            if not self._groupSyncRead.addParam(motor_id):
                raise RuntimeError("[DXL] groupSyncRead.addParam failed to add motor %d" % motor_id)

        # Initialize the packet error flag:
        self._comm_error = 0

        # If the motors are not already configured as desired, set them in velocity control mode and
        # set their velocity limit:
        max_speed = int(max_speed * WheelControl.speed_to_bytes) / WheelControl.speed_to_bytes
        if not all(mode == 1 for mode in self.get_operating_modes()) or not all(
            speed == max_speed for speed in self.get_max_speeds()
        ):
            self.disable()
            self.set_operating_mode(1)
            self.max_speed = max_speed
        else:
            self._max_speed = int(max_speed * WheelControl.speed_to_bytes)

        # Enable the wheel torque:
        self.enable()

    def set_directions(self, reverse_directions):
        """Define the positive direction of rotation for each motor. The argument is a list of
        booleans specifiyng for each motor if its direction has to be reversed."""

        if not isinstance(reverse_directions, collections.abc.Iterable) or len(
            reverse_directions
        ) != len(self._motor_ids):
            raise TypeError(
                "Argument 'speeds' should be a list containing as many booleans as there are motors"
                "to control"
            )

        if not all([isinstance(reverse, bool) for reverse in reverse_directions]):
            raise TypeError("Argument 'reverse_directions' should be a list of booleans")

        self._motor_directions = [-1 if reverse else 1 for reverse in reverse_directions]

    def set_speeds(self, speeds):
        """Send the target speed (in rad/s) for each motor"""

        if not isinstance(speeds, collections.abc.Iterable):
            speeds = [speeds] * len(self._motor_ids)
        elif len(speeds) != len(self._motor_ids):
            raise TypeError(
                "Argument 'speeds' should contains as many setpoints as there are motors to control"
            )

        # Add the target speed value to the sync write parameter storage:
        for i, motor_id in enumerate(self._motor_ids):

            speed = round(self._motor_directions[i] * speeds[i] * WheelControl.speed_to_bytes)
            speed = max(-self._max_speed, min(speed, self._max_speed))

            param_goal_velocity = (
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(speed)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(speed)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(speed)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(speed)),
            )
            if not self._groupSyncWrite.addParam(motor_id, param_goal_velocity):
                raise RuntimeError(
                    "[DXL] groupSyncWrite.addParam failed to add the target speed %f for motor %d"
                    % (speed, motor_id)
                )

        # Sync write the target speeds:
        self._comm_result = self._groupSyncWrite.txPacket()
        self._error_check("[DXL] Failed to write the target speeds: ")

        # Clear syncwrite parameter storage:
        self._groupSyncWrite.clearParam()

    def get_telemetry(self):
        """Return the telemetry as a list of dictionaries, one for each motor, containing the
        present 'velocity' (rad/s), 'current' (mA) and 'voltage' (V)"""

        # Sync read the telemetry:
        self._comm_result = self._groupSyncRead.txRxPacket()
        self._error_check("[DXL] Failed to read telemetry: ")

        telemetry = []

        for motor_id, motor_direction in zip(self._motor_ids, self._motor_directions):

            # Retrieve the raw data:
            current = self._groupSyncRead.getData(
                motor_id, WheelControl.addr_present_current, WheelControl.len_present_current
            )
            velocity = self._groupSyncRead.getData(
                motor_id, WheelControl.addr_present_velocity, WheelControl.len_present_velocity
            )
            position = self._groupSyncRead.getData(
                motor_id, WheelControl.addr_present_position, WheelControl.len_present_position
            )
            voltage = self._groupSyncRead.getData(
                motor_id, WheelControl.addr_present_voltage, WheelControl.len_present_voltage
            )
            temperature = self._groupSyncRead.getData(
                motor_id,
                WheelControl.addr_present_temperature,
                WheelControl.len_present_temperature,
            )
            pwm = self._groupSyncRead.getData(
                motor_id,
                WheelControl.addr_present_pwm,
                WheelControl.len_present_pwm,
            )
            
            # Compute the two's complements to obtain signed values:
            if current > 32768:
                current -= 65536
            if velocity > 2147483648:
                velocity -= 4294967296
            if position > 2147483648:
                position -= 4294967296
            if temperature > 128:
                temperature -= 256
            if pwm > 32768:
                pwm -= 65536

            # Convert the values to the desired units:
            current *= 2.69  # mA
            velocity *= 0.023980823922402087  # rad/s # 0.229*2*pi/60
            position *= 1 / 4096  # rev
            voltage *= 0.1  # V
            pwm *= .113
            # temperature already in deg C

            telemetry.append(
                {
                    "position": motor_direction * position,
                    "velocity": motor_direction * velocity,
                    "current": motor_direction * current,
                    "voltage": voltage,
                    "temperature": temperature,
                    "pwm": pwm,
                }
            )

        return telemetry

    def __del__(self):

        self._portHandler.closePort()
