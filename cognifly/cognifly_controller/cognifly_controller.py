"""
Cognifly onboard controller

TO USE THIS SCRIPT, COGNIFLY MUST BE IN EST_POS DEBUG MODE

Copyright (C) 2021 Yann Bouteiller, Ricardo de Azambuja
"""
import datetime
import struct
import time
import curses
from collections import deque, OrderedDict
from itertools import cycle
import socket
import pickle as pkl
import numpy as np
import math
from pathlib import Path
from yamspy import MSPy
import logging
from threading import Thread, Lock
from abc import ABC, abstractmethod

from cognifly.utils.udp_interface import UDPInterface
from cognifly.utils.tcp_video_interface import TCPVideoInterface
from cognifly.utils.ip_tools import extract_ip
from cognifly.utils.pid import PID
from cognifly.utils.filters import Simple1DKalman, Simple1DExponentialAverage
from cognifly.utils.functions import clip, smallest_angle_diff_rad, is_raspberrypi
from cognifly.utils.joysticks import PS4Gamepad

SWITCH_MODE_CHR = ord('m')
FORWARD_CHR = ord('8')  # curses.KEY_UP
BACKWARD_CHR = ord('5')  # curses.KEY_DOWN
LEFT_CHR = ord('7')  # curses.KEY_LEFT
RIGHT_CHR = ord('9')  # curses.KEY_RIGHT
LEFTYAW_CHR = ord('4')
RIGHTYAW_CHR = ord('6')
UPWARD_CHR = curses.KEY_PPAGE
DOWNWARD_CHR = curses.KEY_NPAGE
PAUSE_CHR = ord('p')
ARM_CHR = ord('a')
DISARM_CHR = ord('d')
REBOOT_CHR = ord('r')
QUIT_CHR = ord('q')
TAKEOFF_CHR = ord('t')
LAND_CHR = ord('l')

KEY_TIMEOUT = 0.2  # seconds before a keyboard command times out

DEFAULT_ROLL = 1500
DEFAULT_PITCH = 1500
DEFAULT_THROTTLE = 900  # throttle bellow a certain value disarms the FC
DEFAULT_YAW = 1500

TAKEOFF = 1300
LAND = 900

MIN_CMD_ROLL = 1250
MIN_CMD_PITCH = 1250
MIN_CMD_THROTTLE = 900  # throttle bellow a certain value disarms the FC
MIN_CMD_V_Z = -1000
MIN_CMD_YAW = 1250
MAX_CMD_ROLL = 1750
MAX_CMD_PITCH = 1750
MAX_CMD_THROTTLE = 1500
MAX_CMD_YAW = 1750
MAX_CMD_V_Z = 1000

KEY_N_ROLL = 1350
KEY_N_PITCH = 1350
KEY_N_THROTTLE = -10  # added on throttle(-) key press
KEY_N_YAW = 1450
KEY_P_ROLL = 1650
KEY_P_PITCH = 1650
KEY_P_THROTTLE = 10  # added on throttle(+) key press
KEY_P_YAW = 1550

KEY_TAKEOFF = TAKEOFF
KEY_LAND = LAND

# DEFAULT_CH5 = 1000  # DISARMED (1000) / ARMED (1800)
# DEFAULT_CH6 = 1500  # ANGLE (1000) / NAV_POSHOLD (1500) <= 1300: NAV ALTHOLD
# DEFAULT_CH7 = 1000  # FAILSAFE (1800)
# DEFAULT_CH8 = 1000  # HEADING HOLD (1800)

ARMED = 1800
DISARMED = 1000
ANGLE_MODE = 1000
NAV_ALTHOLD_MODE = 1500
NAV_POSHOLD_MODE = 1800

DEFAULT_AUX1 = DISARMED  # DISARMED (1000) / ARMED (1800)
DEFAULT_AUX2 = NAV_POSHOLD_MODE  # Angle (1000) / NAV_ALTHOLD (1500) / NAV_POSHOLD (1800)

# Max periods for:
CTRL_LOOP_TIME = 1 / 100
SLOW_MSGS_LOOP_TIME = 1 / 5  # these messages take a lot of time slowing down the loop...
BATT_UDP_MSG_TIME = 1.0

NO_OF_CYCLES_AVERAGE_GUI_TIME = 10
TIME_BETWEEN_CONNECTION_ATTEMPTS = 10

BATT_NO_INFO = -1
BATT_OK = 0
BATT_TOO_HIGH = 1
BATT_WARNING = 2
BATT_TOO_LOW = 3

EPSILON_DIST_TO_TARGET = 0.05  # (m)
EPSILON_ANGLE_TO_TARGET = 1 * np.pi / 180.0  # (rad)


def hover_command():
    return ['PDZ', 0, 0, 0, 0, 0.1, 0.5, time.time() + 1000000.0]


def joystick_to_cmd(value, deadband, default_cmd, min_cmd, max_cmd):
    """
    value between -1.0 and 1.0
    """
    if abs(value) <= deadband / 2:
        return default_cmd
    sum = max_cmd + min_cmd
    dif = max_cmd - min_cmd
    return (sum + value * dif) / 2


def joystick_to_t(value, deadband, max_cmd):
    """
    value between -1.0 and 1.0
    """
    val = (1 + value) / 2
    if abs(val) <= deadband / 2:
        return 0
    return val * max_cmd


def joystick_to_pitch(value, deadband=0.05):
    return joystick_to_cmd(value, deadband, DEFAULT_PITCH, MIN_CMD_PITCH, MAX_CMD_PITCH)


def joystick_to_roll(value, deadband=0.05):
    return joystick_to_cmd(value, deadband, DEFAULT_ROLL, MIN_CMD_ROLL, MAX_CMD_ROLL)


def joystick_to_yaw(value, deadband=0.05):
    return joystick_to_cmd(value, deadband, DEFAULT_YAW, MIN_CMD_YAW, MAX_CMD_YAW)


def trigger_to_positive_vz(value, deadband=0.05):
    return joystick_to_t(value, deadband, MAX_CMD_V_Z)


def trigger_to_negative_vz(value, deadband=0.05):
    return joystick_to_t(value, deadband, MIN_CMD_V_Z)


class PS4GamepadManager:
    def __init__(self):
        self.deadband = 0.05
        self.gamepad = PS4Gamepad()
        self.connected, _, _ = self.gamepad.get()
        self.vz = 0
        self.ts = None
        self.mode = 2
        self.hover = False
        self.z_wait_until = time.time()

    def get(self, CMDS, flight_command):
        """
        Args:
            CMDS: the FC commands
        Returns:
            CMDS: modified FC commands
            emergency: whether to trigger the emergency behavior
            valid: whether the modified commands should be used to override UDP commands
        """
        connected, axis_states, button_states = self.gamepad.get()
        if not connected:
            if self.connected:
                # gamepad has been disconnected, trigger emergency behavior
                self.connected = False
                return CMDS, flight_command, True, True  # trigger emergency
            return CMDS, flight_command, False, False  # not connected
        else:  # connected
            if not self.connected or self.ts is None:  # connection
                self.ts = time.time()
            self.connected = True
            ax = axis_states['x']
            ay = axis_states['y']
            arx = axis_states['rx']
            az = axis_states['z']
            arz = axis_states['rz']
            hatx = axis_states['hat0x']
            haty = axis_states['hat0y']
            ba = button_states['a']
            bb = button_states['b']
            bx = button_states['x']
            by = button_states['y']
            tl = button_states['tl']
            tr = button_states['tr']

            if bx == 1:
                self.mode = 1
                self.ts = time.time()
            elif by == 1:
                self.mode = 0
            elif bb == 1:
                self.mode = 2

            override = False
            now = time.time()

            if tl == tr == 1:
                CMDS['aux1'] = ARMED
                CMDS['throttle'] = DEFAULT_THROTTLE
            elif ba == 1:
                CMDS['aux1'] = DISARMED
                CMDS['throttle'] = DEFAULT_THROTTLE
                override = True

            if haty == -1:
                CMDS['throttle'] = TAKEOFF
                if self.mode == 2:
                    flight_command = ['PDF', 0.0, 0.0, None, None, 0.1, 0.0, time.time() + 1000000.0]
                    self.hover = True
                else:
                    flight_command = None
            elif haty == 1:
                CMDS['throttle'] = LAND
                self.z_wait_until = now + 2.0
                flight_command = None

            z_wait = now < self.z_wait_until

            if not override:
                if self.mode == 1 or z_wait:  # override CMDS
                    CMDS['pitch'] = joystick_to_pitch(- ay, deadband=self.deadband)
                    CMDS['roll'] = joystick_to_roll(ax, deadband=self.deadband)
                    CMDS['yaw'] = joystick_to_yaw(arx, deadband=self.deadband)
                    vz = 0
                    vz += trigger_to_positive_vz(az, deadband=self.deadband)
                    vz += trigger_to_negative_vz(arz, deadband=self.deadband)
                    ts = time.time()
                    CMDS['throttle'] += vz * (ts - self.ts)
                    flight_command = None
                    self.ts = ts
                elif self.mode == 2:  # override flight command
                    vx = joystick_to_cmd(- ay, deadband=self.deadband, default_cmd=0.0, min_cmd=-1.5, max_cmd=1.5)
                    vy = joystick_to_cmd(ax, deadband=self.deadband, default_cmd=0.0, min_cmd=-1.5, max_cmd=1.5)
                    w = joystick_to_cmd(arx, deadband=self.deadband, default_cmd=0.0, min_cmd=-1.0, max_cmd=1.0)
                    vz = 0
                    vz += joystick_to_t(az, deadband=self.deadband, max_cmd=0.2)
                    vz -= joystick_to_t(arz, deadband=self.deadband, max_cmd=0.2)
                    if 0 == vx == vy == vz == w:
                        if not self.hover:  # we have to send position command PDZ only once!
                            flight_command = hover_command()
                            self.hover = True
                    else:  # send velocity command
                        self.hover = False
                        flight_command = ['VDF', vx, vy, vz, w, time.time() + 1.0]
        return CMDS, flight_command, False, self.mode


class SignalTracer:
    """
    This can be used to trace the drone dynamics in real-time (e.g. to output a CSV-like log file)
    """
    def __init__(self, filepath):
        self.file = open(filepath, 'w+')

    def write_line(self, line):
        self.file.write(line + '\n')

    def __del__(self):
        self.file.close()


def try_connect(drone_hostname, drone_port):
    try:
        udp_int = UDPInterface()
        drone_hostname_ = socket.gethostname() if drone_hostname is None else drone_hostname
        drone_ip = socket.gethostbyname(drone_hostname_) if drone_hostname is not None else extract_ip()
        if drone_hostname is None and (drone_ip == '127.0.0.1' or drone_ip == '0.0.0.0'):
            raise RuntimeError(f"Could not extract drone IP ({drone_ip})")
        udp_int.init_receiver(ip=drone_ip, port=drone_port)
        tcp_video_int = TCPVideoInterface()
    except Exception as e:
        udp_int = None
        tcp_video_int = None
        drone_hostname_ = None
        drone_ip = None
    return drone_hostname_, drone_ip, drone_port, udp_int, tcp_video_int


def try_addstr(*args):
    screen = args[0]
    args_addstr = args[1:]
    try:
        screen.addstr(*args_addstr)
    except Exception as e:
        pass


class PoseEstimator(ABC):
    """
    Base interface for custom pose estimators
    """
    @abstractmethod
    def get(self):
        """
        Must return a tuple of 8 values: (pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate)
        If any is None, this is considered failure and the onboard estimator will be used instead.

        These values represent the drone attitude in the world frame:
        pos_x_wf = x position (m)
        pos_y_wf = y position (m)
        pos_z_wf = z position (m)
        yaw: yaw (rad)
        vel_x_wf = x velocity (m/s)
        vel_y_wf = y velocity (m/s)
        vel_z_wf = z velocity (m/s)
        yaw_rate: yaw rate (rad/s)

        CAUTION: your estimator needs to respect the cognifly coordinate system, which is not standard:
        x: FORWARD
        y: RIGHT
        z: UP
        """
        raise NotImplementedError

    def get_fc_estimate(self):
        """
        Use this method if you wish to access the internal estimate of the flight controller
        This method is redefined when run_curses() is executed in the CogniflyController object
        Until then, it returns None for all estimates
        """
        return None, None, None, None, None, None, None, None


def set_gps(board,
            longitude,
            latitude,
            mslAltitude,
            nedVelNorth=0,
            nedVelEast=0,
            nedVelDown=0,
            groundCourse=0):
    if mslAltitude < 0 or mslAltitude > 2147483647:
        logging.warning(f'GPS altitude cannot be < 0')
        mslAltitude = 0
    msp2_gps_format = '<BHIBBHHHHiiiiiiHHHBBBBB'  # https://docs.python.org/3/library/struct.html#format-characters
    now = datetime.datetime.now()
    gps_data = {
        'instance': 1,  # uint8 -  sensor instance number to support multi-sensor setups
        'gpsWeek': 0xFFFF,  # uint16 - GPS week, 0xFFFF if not available
        'msTOW': 0,  # uint32
        'fixType': 10,  # uint8
        'satellitesInView': 10,  # uint8
        'horizontalPosAccuracy': 1,  # uint16 - [cm]
        'verticalPosAccuracy': 1,  # uint16 - [cm]
        'horizontalVelAccuracy': 1,  # uint16 - [cm/s]
        'hdop': 100,  # uint16
        'longitude': longitude * 10000000,  # int32
        'latitude': latitude * 10000000,  # int32
        'mslAltitude': mslAltitude,  # int32 - [cm]
        'nedVelNorth': nedVelNorth,  # int32 - [cm/s]
        'nedVelEast': nedVelEast,  # int32
        'nedVelDown': nedVelDown,  # int32
        'groundCourse': groundCourse,  # uint16 - deg * 100, 0..36000
        'trueYaw': 65535,  # uint16 - deg * 100, values of 0..36000 are valid. 65535 = no data available
        'year': now.year,  # uint16
        'month': now.month,  # uint8
        'day': now.day,  # uint8
        'hour': now.hour,  # uint8
        'min': now.minute,  # uint8
        'sec': now.second,  # uint8
    }
    data = struct.pack(msp2_gps_format, *[int(i) for i in gps_data.values()])
    board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_GPS'], data=data, flush=False)


def set_gps_from_xyz(board, x, y, z, vx=0, vy=0, vz=0):
    """
    x is north, y is east, z is up, from longitude 0, latitude 0 and sea level.
    """
    latitude = x / 111000.0
    longitude = y / 111000.0
    mslAltitude = z * 100.0
    set_gps(board, longitude, latitude, mslAltitude)


def set_compass(board, magX, magY, magZ, t_start):
    msp2_compass_format = '<BIhhh'  # https://docs.python.org/3/library/struct.html#format-characters
    compass_data = {
        'instance': 2,  # uint8 -  sensor instance number to support multi-sensor setups
        'timeMs': round((time.time() - t_start) * 1000),  # uint32
        'magX': round(magX),  # int16_t mGauss, front
        'magY': round(magY),  # int16_t mGauss, right
        'magZ': round(magZ)  # int16_t mGauss, down
    }
    data = struct.pack(msp2_compass_format, *[int(i) for i in compass_data.values()])
    board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_COMPASS'], data=data, flush=False)


def set_compass_from_yaw(board, yaw, t_start):
    x = np.cos(yaw) * 32767
    y = np.sin(yaw) * 32767
    set_compass(board, x, y, 0, t_start)


def set_barometer(board, pressurePa, temp, t_start):
    msp2_baro_format = '<BIfh'  # https://docs.python.org/3/library/struct.html#format-characters
    barometer_data = {
        'instance': 3,  # uint8 -  sensor instance number to support multi-sensor setups
        'timeMs': round((time.time() - t_start) * 1000),  # uint32
        'pressurePa': pressurePa,  # float
        'temp': round(temp),  # int16_t centi-degrees C
    }
    data = struct.pack(msp2_baro_format, *barometer_data.values())
    board.send_RAW_msg(MSPy.MSPCodes['MSP2_SENSOR_BAROMETER'], data=data, flush=False)


def set_barometer_from_altitude(board, altitude, t_start):
    temp = 2500  # centi-degrees C
    pressure_pa = ((1.0 - (altitude / 44330.0)) ** 5.254999) * 101325.0
    set_barometer(board, pressure_pa, temp, t_start)


class CogniflyController:
    def __init__(self,
                 network=True,
                 drone_hostname=None,
                 drone_port=8988,
                 print_screen=True,
                 obs_loop_time=0.1,
                 trace_logs=False,
                 pose_estimator=None,
                 pid_limit=500,
                 vel_x_kp=750.0,
                 vel_x_ki=400.0,
                 vel_x_kd=50.0,
                 vel_y_kp=750.0,
                 vel_y_ki=400.0,
                 vel_y_kd=50.0,
                 vel_z_kp=50.0,
                 vel_z_ki=20.0,
                 vel_z_kd=0.0,  # 5.0
                 vel_w_kp=400.0,
                 vel_w_ki=200.0,
                 vel_w_kd=0.0,
                 x_vel_gain=0.5,
                 y_vel_gain=0.5,
                 z_vel_gain=0.2,
                 w_gain=0.5,
                 custom_gps=True,
                 custom_compass=True,
                 custom_barometer=True,
                 custom_debug_mode=False):
        """
        Custom controller and udp interface for Cognifly
        Args:
            network: bool: if False, the drone will not run a udp server (True is needed for remote control)
            drone_hostname: str (optional): can be an IP. If None, the ip is extracted automatically.
            drone_port: int (optional): port used to receive commands from the udp controller
            print_screen: bool (optional):
                if True, messages will be printed repeatedly on the local screen using curses
                if False, only the key presses will be read repeatedly using curses
            obs_loop_time: float (optional):
                if None, an observation is sent by UDP as answer each time a UDP command is received
                else, an observation is sent bu UDP every obs_loop_time seconds
            trace_logs: bool (optional): if True, flight telemetry will be printed in a CSV-like log file
            pose_estimator: cognifly.cognifly_controller.cognifly_controller.PoseEstimator: custom pose estimator
        """
        self.custom_debug_mode = custom_debug_mode
        self.pose_estimator = pose_estimator
        if self.pose_estimator is None:
            self.custom_gps = False
            self.custom_compass = False
            self.custom_barometer = False
        else:
            self.custom_gps = custom_gps
            self.custom_compass = custom_compass
            self.custom_barometer = custom_barometer
        self.board = None
        self.valid_custom_estimate = True
        self.network = network
        self.drone_hostname = drone_hostname
        self.drone_port = drone_port
        self.udp_int = None
        self.tcp_video_int = None
        self.drone_ip = None
        self._armed = False
        self._lock_connect = Lock()
        self._drone_hostname = drone_hostname
        self._drone_ip = self.drone_ip
        self._drone_port = drone_port
        self._udp_int = self.udp_int
        self._tcp_video_int = self.tcp_video_int
        if self.network:
            self._t_connect = Thread(target=self._try_connect_thread, args=(drone_hostname, drone_port), daemon=True)
            self._t_connect.start()
            # self.drone_hostname, self.drone_ip, self.drone_port, self.udp_int, self.tcp_video_int = try_connect(drone_hostname, drone_port)
        self.sender_initialized = False  # sender is initialized only when a reset message is received
        self.print_screen = print_screen
        self.obs_loop_time = obs_loop_time
        self.voltage = None
        self.min_voltage = None
        self.warn_voltage = None
        self.max_voltage = None
        self.key_cmd_in_progress = False
        self.udp_armed = False
        self.ludp = 0
        self.last_key_tick = time.time()
        self.last_obs_tick = time.time()
        self.last_bat_tick = time.time()
        # reminder:
        # 'CH5': DEFAULT_CH5  # DISARMED (1000) / ARMED (1800)
        # 'CH6': DEFAULT_CH6  # ANGLE (1000) / NAV_POSHOLD (1500)
        # 'CH7': DEFAULT_CH7  # FAILSAFE (1800)
        # 'CH8': DEFAULT_CH8  # HEADING HOLD (1800)
        self.CMDS = {'roll': DEFAULT_ROLL,
                     'pitch': DEFAULT_PITCH,
                     'throttle': DEFAULT_THROTTLE,  # throttle bellow a certain value disarms the FC
                     'yaw': DEFAULT_YAW,
                     'aux1': DEFAULT_AUX1,
                     'aux2': DEFAULT_AUX2
                     }
        self.CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
        self.__batt_state = BATT_NO_INFO

        self.current_flight_command = None
        self._flight_origin = None
        self.prev_yaw_prime_ts = None
        self.prev_alt_prime_ts = None

        self.telemetry = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.debug_flags = []
        self._i_obs = 0
        self.target_flag = False
        self.target_id = None

        # PID values:

        self.pid_limits_xy = (-pid_limit, +pid_limit)
        self.pid_limits_z = (-pid_limit, +pid_limit)
        self.pid_limits_w = (-pid_limit, +pid_limit)

        self.vel_x_kp = vel_x_kp
        self.vel_x_ki = vel_x_ki
        self.vel_x_kd = vel_x_kd

        self.vel_y_kp = vel_y_kp
        self.vel_y_ki = vel_y_ki
        self.vel_y_kd = vel_y_kd

        self.vel_z_kp = vel_z_kp
        self.vel_z_ki = vel_z_ki
        self.vel_z_kd = vel_z_kd

        self.vel_w_kp = vel_w_kp
        self.vel_w_ki = vel_w_ki
        self.vel_w_kd = vel_w_kd

        self.pid_vel_x = None
        self.pid_vel_y = None
        self.pid_vel_z = None
        self.pid_w_z = None

        self.x_vel_gain = x_vel_gain
        self.y_vel_gain = y_vel_gain
        self.z_vel_gain = z_vel_gain
        self.w_gain = w_gain

        self.last_cmd_type = None
        self._reset_pids()

        # filters:

        # self.filter_vel_z = Simple1DKalman(err_measure=0.3, q=0.9)
        # self.filter_w_z = Simple1DKalman(err_measure=0.2, q=0.9)
        self.filter_vel_z = Simple1DExponentialAverage(tau=0.2)
        self.filter_w_z = Simple1DExponentialAverage(tau=0.2)

        # tracer:

        self.start_time = time.time()
        self.trace_logs = trace_logs
        if self.trace_logs:
            self.w_tracer = SignalTracer(Path(__file__).parent.resolve() / "trace_logs" / "w_trace.log")
            self.z_tracer = SignalTracer(Path(__file__).parent.resolve() / "trace_logs" / "z_trace.log")
            self.x_tracer = SignalTracer(Path(__file__).parent.resolve() / "trace_logs" / "x_trace.log")
            self.y_tracer = SignalTracer(Path(__file__).parent.resolve() / "trace_logs" / "y_trace.log")
        else:
            self.w_tracer = None
            self.z_tracer = None
            self.x_tracer = None
            self.y_tracer = None

        # PS4 controller:

        self.gamepad_manager = PS4GamepadManager()

        # Others:

        self.emergency = False
        self._t_start = time.time()
        self.device_str = "/dev/ttyS0" if is_raspberrypi() else "/dev/ttyS1"
        self._alpha_d = 0.1
        self._d1 = 0
        self._d2 = 0
        self._d3 = 0

    def _try_connect_thread(self, drone_hostname, drone_port):
        _drone_hostname, _drone_ip, _drone_port, _udp_int, _tcp_video_int = None, None, None, None, None
        while _udp_int is None:
            _drone_hostname, _drone_ip, _drone_port, _udp_int, _tcp_video_int = try_connect(drone_hostname, drone_port)
            if _udp_int is None:
                time.sleep(TIME_BETWEEN_CONNECTION_ATTEMPTS)
        with self._lock_connect:
            self._drone_hostname = _drone_hostname
            self._drone_ip = _drone_ip
            self._drone_port = _drone_port
            self._udp_int = _udp_int
            self._tcp_video_int = _tcp_video_int

    def run_curses(self, reboot_fc=True, profiling_duration=-1):
        """
        Runs the controller.

        Args:
            reboot_fc: bool: if True, the flight controller will first reboot.
            profiling_duration: float: if >= 0, profile code for this number of seconds.
        """
        if reboot_fc:
            with MSPy(device=self.device_str, loglevel='WARNING', baudrate=115200) as board:
                if board == 1:  # an error occurred...
                    raise RuntimeError('board unavailable')
                else:
                    print("Rebooting flight controller...")
                    board.reboot()
                    time.sleep(5)
                    print("Flight controller ready.")
        try:
            if self.print_screen:
                # get the curses screen window
                screen = curses.initscr()
                # turn off input echoing
                curses.noecho()
                # respond to keys immediately (don't wait for enter)
                curses.cbreak()
                # non-blocking
                screen.timeout(0)
                # map arrow keys to special values
                screen.keypad(True)
                try_addstr(screen, 1, 0, "Press 'q' to quit, 'r' to reboot, 'm' to change mode, 'a' to arm, 'd' to disarm and arrow keys to control", curses.A_BOLD)
            else:
                screen = None
            result = self._controller(screen, profiling_duration=profiling_duration)
        finally:
            # shut down cleanly
            if self.print_screen:
                curses.nocbreak()
                screen.keypad(0)
                curses.echo()
                curses.endwin()
            if result == 1:
                print("An error occurred, probably the serial port is not available")
            return result

    def _reset_pids(self):
        """
        Resets all PIDs
        """
        self.pid_vel_x = PID(kp=self.vel_x_kp, ki=self.vel_x_ki, kd=self.vel_x_kd, sample_time=0.01, output_limits=self.pid_limits_xy, auto_mode=False)
        self.pid_vel_y = PID(kp=self.vel_y_kp, ki=self.vel_y_ki, kd=self.vel_y_kd, sample_time=0.01, output_limits=self.pid_limits_xy, auto_mode=False)
        self.pid_vel_z = PID(kp=self.vel_z_kp, ki=self.vel_z_ki, kd=self.vel_z_kd, sample_time=0.01, output_limits=self.pid_limits_z, auto_mode=False)
        self.pid_w_z = PID(kp=self.vel_w_kp, ki=self.vel_w_ki, kd=self.vel_w_kd, sample_time=0.01, output_limits=self.pid_limits_w, auto_mode=False)

    def _udp_commands_handler(self, command):
        """
        Args:
            command: a command of the form (str:command, ...:args)
        """
        if command[0] == "RES":  # reset drone
            if self.CMDS['aux1'] != DISARMED:
                self.emergency = True  # land if not disarmed
            self.CMDS['roll'] = DEFAULT_ROLL
            self.CMDS['pitch'] = DEFAULT_PITCH
            self.CMDS['throttle'] = DEFAULT_THROTTLE  # throttle bellow a certain value disarms the FC
            self.CMDS['yaw'] = DEFAULT_YAW
            self.CMDS['aux2'] = DEFAULT_AUX2
            self.udp_int.init_sender(ip=command[2][0], port=command[2][1])
            self.sender_initialized = True
            self._flight_origin = None
            self.udp_int.send(pkl.dumps(("RES", (self.drone_ip, self.drone_port))))
            self.current_flight_command = None
            self._reset_pids()
        elif self.sender_initialized:  # do not take any udp order unless reset has been called
            identifier = command[1]
            if command[0] == "ACT":  # action
                if self.last_cmd_type != command[2][0]:
                    self._reset_pids()
                    self.last_cmd_type = command[2][0]
                if command[2][0] == "DISARM":
                    self.CMDS["aux1"] = DISARMED
                    self.CMDS["throttle"] = DEFAULT_THROTTLE
                    self.current_flight_command = None
                elif command[2][0] == "ARM":
                    self.CMDS["aux1"] = ARMED
                    self.CMDS["aux2"] = NAV_POSHOLD_MODE
                    self.CMDS["throttle"] = DEFAULT_THROTTLE
                    self.current_flight_command = None
                elif command[2][0] == "TAKEOFF":
                    alt = command[2][1] if command[2][1] is not None else TAKEOFF
                    track_xy = command[2][2]
                    max_duration = command[2][3]
                    max_velocity = command[2][4]
                    self.CMDS["throttle"] = alt  # TODO: replace by true altitude (now a throttle value)
                    self.CMDS["aux2"] = NAV_POSHOLD_MODE
                    if track_xy:
                        self.current_flight_command = ["PDF", 0.0, 0.0, None, None, max_velocity, 0.0, time.time() + max_duration]
                    else:
                        self.current_flight_command = None
                elif command[2][0] == "LAND":
                    self.CMDS["throttle"] = LAND
                    self.CMDS["aux2"] = NAV_POSHOLD_MODE
                    self.current_flight_command = None
                elif command[2][0] in ("VDF", "VWF"):  # velocity
                    self.current_flight_command = [command[2][0], command[2][1], command[2][2], command[2][3], command[2][4], time.time() + command[2][5]]
                elif command[2][0] in ("PDF", "PDZ", "PWF"):  # position
                    self.target_flag = True
                    self.target_id = identifier
                    self.current_flight_command = [command[2][0], command[2][1], command[2][2], command[2][3], command[2][4], command[2][5], command[2][6], time.time() + command[2][7]]
            elif command[0] == "ST1":  # stream on
                if self.tcp_video_int is not None:
                    self.tcp_video_int.start_streaming(ip_dest=command[2][0], port_dest=command[2][1], resolution=command[2][2], fps=command[2][3], compress_format=command[2][4], compress_quality=command[2][5])
            elif command[0] == "ST0":  # stream off
                if self.tcp_video_int is not None:
                    self.tcp_video_int.stop_streaming()
            elif command[0] == "PVX":  # new PID vel x values
                if command[2][0] is not None:
                    self.vel_x_kp = command[2][0]
                if command[2][1] is not None:
                    self.vel_x_ki = command[2][1]
                if command[2][2] is not None:
                    self.vel_x_kd = command[2][2]
                self._reset_pids()
            elif command[0] == "PVY":  # new PID vel y values
                if command[2][0] is not None:
                    self.vel_y_kp = command[2][0]
                if command[2][1] is not None:
                    self.vel_y_ki = command[2][1]
                if command[2][2] is not None:
                    self.vel_y_kd = command[2][2]
                self._reset_pids()
            elif command[0] == "PVZ":  # new PID vel z values
                if command[2][0] is not None:
                    self.vel_z_kp = command[2][0]
                if command[2][1] is not None:
                    self.vel_z_ki = command[2][1]
                if command[2][2] is not None:
                    self.vel_z_kd = command[2][2]
                self._reset_pids()
            elif command[0] == "PVW":  # new PID vel w values
                if command[2][0] is not None:
                    self.vel_w_kp = command[2][0]
                if command[2][1] is not None:
                    self.vel_w_ki = command[2][1]
                if command[2][2] is not None:
                    self.vel_w_kd = command[2][2]
                self._reset_pids()
            elif command[0] == "SCO":  # set current drone position (XY) and yaw
                cur_x_wf = self.telemetry[0]
                cur_y_wf = self.telemetry[1]
                cur_yaw = self.telemetry[3]
                fo = (0, 0, 0) if self._flight_origin is None else self._flight_origin
                new_x_wf = command[2][0] if command[2][0] is not None else cur_x_wf
                new_y_wf = command[2][1] if command[2][1] is not None else cur_y_wf
                new_yaw = command[2][2] if command[2][2] is not None else cur_yaw
                new_fo_yaw = fo[2] + cur_yaw - new_yaw
                new_fo_yaw = smallest_angle_diff_rad(new_fo_yaw, 0.0)
                new_fo_x = fo[0] + cur_x_wf * np.cos(fo[2]) - cur_y_wf * np.sin(fo[2]) - new_x_wf * np.cos(new_fo_yaw) + new_y_wf * np.sin(new_fo_yaw)
                new_fo_y = fo[1] + cur_x_wf * np.sin(fo[2]) + cur_y_wf * np.cos(fo[2]) - new_x_wf * np.sin(new_fo_yaw) - new_y_wf * np.cos(new_fo_yaw)
                self._flight_origin = (new_fo_x, new_fo_y, new_fo_yaw)
                self._reset_pids()
                if self.current_flight_command is not None and self.pose_estimator is not None:  # if not on the ground
                    self.current_flight_command = hover_command()
                else:
                    self.current_flight_command = None
            elif command[0] == "SFO":  # directly set flight origin
                fo = (0, 0, 0) if self._flight_origin is None else self._flight_origin
                new_fo_x = command[2][0] if command[2][0] is not None else fo[0]
                new_fo_y = command[2][1] if command[2][1] is not None else fo[1]
                new_fo_yaw = command[2][2] if command[2][2] is not None else fo[2]
                new_fo_yaw = smallest_angle_diff_rad(new_fo_yaw, 0.0)
                self._flight_origin = (new_fo_x, new_fo_y, new_fo_yaw)
                self._reset_pids()
                if self.current_flight_command is not None and self.pose_estimator is not None:  # if not on the ground
                    self.current_flight_command = hover_command()
                else:
                    self.current_flight_command = None
            self.udp_int.send(pkl.dumps(("ACK", identifier)))

    def _compute_z_vel(self, pos_z_wf):
        if self.prev_alt_prime_ts is None:
            self.prev_alt_prime_ts = (pos_z_wf, 0.0, time.time())
            res = 0.0
        else:
            now = time.time()
            dt = now - self.prev_alt_prime_ts[2]
            if dt <= 0:
                res = self.prev_alt_prime_ts[1]
            else:
                res = (pos_z_wf - self.prev_alt_prime_ts[0]) / dt
                self.prev_alt_prime_ts = (pos_z_wf, res, now)
        before_filter = res
        res = self.filter_vel_z.update_estimate(res)
        return res

    def _compute_yaw_rate(self, yaw):
        if self.prev_yaw_prime_ts is None:
            self.prev_yaw_prime_ts = (yaw, 0.0, time.time())
            res = 0.0
        else:
            now = time.time()
            dt = now - self.prev_yaw_prime_ts[2]
            if dt <= 0:
                res = self.prev_yaw_prime_ts[1]
            else:
                # to handle jumps in yaw, we take the smallest abolute value
                diff = smallest_angle_diff_rad(yaw, self.prev_yaw_prime_ts[0])
                res = diff / dt
                self.prev_yaw_prime_ts = (yaw, res, now)
        before_filter = res
        res = self.filter_w_z.update_estimate(res)
        return res

    def _read_estimate(self, board):
        """
        Reads estimates from board
        """
        board.fast_read_attitude()
        yaw = board.SENSOR_DATA['kinematics'][2] * np.pi / 180.0
        if self.custom_debug_mode:
            board.send_RAW_msg(MSPy.MSPCodes['MSP_DEBUG'])  # MSP2_INAV_DEBUG is too long to answer
            data_handler = board.receive_msg()
            board.process_recv_data(data_handler)
            pos_x_wf = board.SENSOR_DATA['debug'][0] / 1e4
            pos_y_wf = board.SENSOR_DATA['debug'][1] / 1e4
            vel_x_wf = board.SENSOR_DATA['debug'][2] / 1e4
            vel_y_wf = board.SENSOR_DATA['debug'][3] / 1e4
        else:
            pos_x_wf = None
            pos_y_wf = None
            vel_x_wf = None
            vel_y_wf = None
        board.fast_read_altitude()
        pos_z_wf = board.SENSOR_DATA['altitude']
        yaw_rate = self._compute_yaw_rate(yaw)
        vel_z_wf = self._compute_z_vel(pos_z_wf)
        return pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate

    def _read_board(self):
        board = self.board
        if board is None:
            return None, None, None, None, None, None, None, None
        else:
            return self._read_estimate(board)

    def _flight(self, board, screen):
        """
        This method is a high-level flight controller.
        Depending on the current flight command and state of the drone, it outputs relevant MSP commands.
        This is done thanks to python PIDs and filters.
        In the future, most of this should be integrated directly in the inav flight controller.
        Indeed, the use of PIDs here is redundant with what happens in inav.
        However, the cognifly version of inav will not take position/velocities as input at the moment but only RPYT, hence this method.
        The use of PIDs for velocities should be removed fairly easily, if we can find the mapping from velocities to RPYT.
            This is the inverse of a monotonic polynomial for x and y, see inav code (potentially no closed form?).
            We have not found the equivalent for z and yaw in inav code yet.
        """
        # retrieve the current state
        failure_custom = False
        recovery = False  # when this flag becomes True, we switch estimators and must change the flight origin
        if self.pose_estimator is not None:  # TODO: test this part of the code (including recovery)
            pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate = self.pose_estimator.get()
            if None in (pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate):
                failure_custom = True
            if failure_custom and self.valid_custom_estimate:
                self.valid_custom_estimate = False
                recovery = True
                if self.print_screen:
                    try_addstr(screen, 28, 0, f"Custom estimate: INVALID")
                    screen.clrtoeol()
            elif not failure_custom and not self.valid_custom_estimate:
                self.valid_custom_estimate = True
                recovery = True
                if self.print_screen:
                    try_addstr(screen, 28, 0, f"Custom estimate: valid")
                    screen.clrtoeol()
        if self.pose_estimator is None or failure_custom:
            if self.custom_debug_mode:
                pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate = self._read_estimate(board)
            else:
                # cannot retrieve from the FC if custom_debug_mode is not active!
                self.current_flight_command = None
                return
        else:
            t_s = time.time()
            if self.custom_barometer:
                set_barometer_from_altitude(board=board, altitude=pos_z_wf, t_start=self._t_start)
            self._d1 = (1 - self._alpha_d) * self._d1 + self._alpha_d * (time.time() - t_s)
            t_s = time.time()
            if self.custom_gps:
                set_gps_from_xyz(board=board, x=pos_x_wf, y=pos_y_wf, z=pos_z_wf)
            self._d2 = (1 - self._alpha_d) * self._d2 + self._alpha_d * (time.time() - t_s)
            t_s = time.time()
            if self.custom_compass:
                set_compass_from_yaw(board=board, yaw=yaw, t_start=self._t_start)
            self._d3 = (1 - self._alpha_d) * self._d3 + self._alpha_d * (time.time() - t_s)


        old_pos_x_wf = pos_x_wf
        old_pos_y_wf = pos_y_wf
        old_vel_x_wf = vel_x_wf
        old_vel_y_wf = vel_y_wf
        old_yaw = yaw

        if self._flight_origin is None:  # new basis at reset
            self._flight_origin = (pos_x_wf, pos_y_wf, yaw)
        else:
            # change of basis
            # NB: this is only to avoid rebooting the board at reset
            x_int = pos_x_wf - self._flight_origin[0]
            y_int = pos_y_wf - self._flight_origin[1]
            cos = np.cos(self._flight_origin[2])
            sin = np.sin(self._flight_origin[2])
            pos_x_wf = x_int * cos + y_int * sin
            pos_y_wf = y_int * cos - x_int * sin
            vel_x_int = vel_x_wf
            vel_y_int = vel_y_wf
            vel_x_wf = vel_x_int * cos + vel_y_int * sin
            vel_y_wf = vel_y_int * cos - vel_x_int * sin
            yaw = smallest_angle_diff_rad(yaw, self._flight_origin[2])

            if recovery:  # change the origin to compensate for the glitch between estimators
                fo_init = self._flight_origin
                fo_pos_x_wf, fo_pos_y_wf, fo_yaw = self._flight_origin
                fo_pos_x_wf += pos_x_wf - self.telemetry[0]
                fo_pos_y_wf += pos_y_wf - self.telemetry[1]
                fo_yaw += smallest_angle_diff_rad(yaw, self.telemetry[3])
                fo_yaw = smallest_angle_diff_rad(fo_yaw, 0.0)  # FIXME: check that this works
                self._flight_origin = (fo_pos_x_wf, fo_pos_y_wf, fo_yaw)
                if self.print_screen:
                    try_addstr(screen, 29, 0, f"FO changed from{fo_init} to {self._flight_origin}")
                    screen.clrtoeol()
                return

        cos = np.cos(yaw)
        sin = np.sin(yaw)
        vel_z_df = vel_z_wf
        vel_x_df = vel_x_wf * cos + vel_y_wf * sin
        vel_y_df = vel_y_wf * cos - vel_x_wf * sin

        if self.print_screen:
            try_addstr(screen, 18, 0, f"yaw: {yaw/np.pi: .5f} pi rad")
            try_addstr(screen, 19, 0, f"yaw rate: {yaw_rate/np.pi: .5f} pi rad/s")
            try_addstr(screen, 20, 0, f"pos_wf: [{pos_x_wf: .5f},{pos_y_wf: .5f},{pos_z_wf: .5f}] m")
            try_addstr(screen, 21, 0, f"vel_wf: [{vel_x_wf: .5f},{vel_y_wf: .5f},{vel_z_wf: .5f}] m/s")
            try_addstr(screen, 22, 0, f"vel_df: [{vel_x_df: .5f},{vel_y_df: .5f},{vel_z_df: .5f}] m/s")
            try_addstr(screen, 24, 0, f"from yaw: {old_yaw / np.pi: .5f} pi rad")
            try_addstr(screen, 25, 0, f"from pos_wf: [{old_pos_x_wf: .5f},{old_pos_y_wf: .5f},{pos_z_wf: .5f}] m")
            try_addstr(screen, 26, 0, f"from vel_wf: [{old_vel_x_wf: .5f},{old_vel_y_wf: .5f},{vel_z_wf: .5f}] m/s")
            screen.clrtoeol()

        self.telemetry = (pos_x_wf, pos_y_wf, pos_z_wf, yaw, vel_x_wf, vel_y_wf, vel_z_wf, yaw_rate)

        if self.current_flight_command is None:
            # ensure that rpy commands are at default:
            self.CMDS['pitch'] = DEFAULT_PITCH
            self.CMDS['roll'] = DEFAULT_ROLL
            self.CMDS['yaw'] = DEFAULT_YAW
        else:
            # handle the current command
            if self.current_flight_command[0] in ("VDF", "VWF"):  # velocity drone/world frame command
                # command is ["VXF", x, y, z, time_end]
                time_end = self.current_flight_command[5]
                if time.time() >= time_end:
                    if self.pose_estimator is None:
                        # stop moving and remove flight command
                        self.pid_vel_x.set_auto_mode(False)
                        self.pid_vel_y.set_auto_mode(False)
                        self.pid_vel_z.set_auto_mode(False)
                        self.pid_w_z.set_auto_mode(False)
                        self.CMDS['roll'] = DEFAULT_ROLL
                        self.CMDS['yaw'] = DEFAULT_YAW
                        self.current_flight_command = None
                    else:
                        # track current position
                        self.current_flight_command = hover_command()
                else:
                    if self.current_flight_command[0] == "VDF":  # drone frame
                        v_x = self.current_flight_command[1]
                        v_y = self.current_flight_command[2]
                    else:  # world frame
                        v_x_wf = self.current_flight_command[1]
                        v_y_wf = self.current_flight_command[2]
                        v_x = v_x_wf * cos + v_y_wf * sin
                        v_y = v_y_wf * cos - v_x_wf * sin
                    v_z = self.current_flight_command[3]
                    w = self.current_flight_command[4]
                    # adapt pitch, roll and throttle
                    self.pid_vel_x.setpoint = v_x
                    self.pid_vel_y.setpoint = v_y
                    self.pid_vel_z.setpoint = v_z
                    self.pid_w_z.setpoint = w
                    self.pid_vel_x.set_auto_mode(True)
                    self.pid_vel_y.set_auto_mode(True)
                    if v_z != 0:
                        self.pid_vel_z.set_auto_mode(True)
                    else:
                        self.pid_vel_z.set_auto_mode(False)
                    if w != 0:
                        self.pid_w_z.set_auto_mode(True)
                    else:
                        self.pid_w_z.set_auto_mode(False)
                    x_target = self.pid_vel_x(vel_x_df)
                    y_target = self.pid_vel_y(vel_y_df)
                    z_target = self.pid_vel_z(vel_z_df) if v_z != 0 else 0
                    w_target = self.pid_w_z(yaw_rate) if w != 0 else 0
                    self.CMDS['pitch'] = DEFAULT_PITCH + x_target
                    self.CMDS['roll'] = DEFAULT_ROLL + y_target
                    self.CMDS['throttle'] = self.CMDS['throttle'] + z_target
                    self.CMDS['yaw'] = DEFAULT_YAW + w_target
            elif self.current_flight_command[0] == "PWF":  # position command
                # command is ["PWF", x, y, z, yaw, vel_norm_goal, w_norm_goal, duration]
                time_end = self.current_flight_command[7]
                if time.time() >= time_end:
                    if self.pose_estimator is None:
                        # stop moving and remove flight command
                        self.pid_vel_x.set_auto_mode(False)
                        self.pid_vel_y.set_auto_mode(False)
                        self.pid_vel_z.set_auto_mode(False)
                        self.pid_w_z.set_auto_mode(False)
                        self.CMDS['pitch'] = DEFAULT_PITCH
                        self.CMDS['roll'] = DEFAULT_ROLL
                        self.CMDS['yaw'] = DEFAULT_YAW
                        self.current_flight_command = None
                    else:
                        # track current position
                        self.current_flight_command = hover_command()
                else:
                    # compute a 4D vector toward the goal:
                    x_goal = self.current_flight_command[1]
                    y_goal = self.current_flight_command[2]
                    z_goal = self.current_flight_command[3]
                    yaw_goal = self.current_flight_command[4]
                    vel_norm_goal = self.current_flight_command[5]
                    w_norm_goal = self.current_flight_command[6]
                    x_vector = x_goal - pos_x_wf
                    y_vector = y_goal - pos_y_wf
                    z_vector = z_goal - pos_z_wf if z_goal is not None else 0.0
                    yaw_vector = smallest_angle_diff_rad(yaw_goal, yaw) if yaw_goal is not None else None
                    if self.target_flag:  # check whether position target is reached
                        dist_condition = np.linalg.norm([x_vector, y_vector, z_vector]) <= EPSILON_DIST_TO_TARGET
                        angle_condition = True if yaw_vector is None else abs(yaw_vector) <= EPSILON_ANGLE_TO_TARGET
                        if dist_condition and angle_condition:
                            self.target_flag = False  # disable the flag
                            if self.sender_initialized:
                                self.udp_int.send(pkl.dumps(("DON", self.target_id)))  # notify the remote control
                    # convert x and y to the drone frame:
                    x_vector_df = x_vector * cos + y_vector * sin
                    y_vector_df = y_vector * cos - x_vector * sin
                    # compute the corresponding desired velocity (non-clipped):
                    v_x = self.x_vel_gain * x_vector_df
                    v_y = self.y_vel_gain * y_vector_df
                    v_z = self.z_vel_gain * z_vector if z_goal is not None else 0.0
                    w = self.w_gain * yaw_vector if yaw_goal is not None else 0.0
                    # clip to desired velocty:
                    vel_norm = np.linalg.norm([v_x, v_y, v_z])
                    w_norm = abs(w)
                    if vel_norm > vel_norm_goal:
                        v_x = v_x * vel_norm_goal / vel_norm
                        v_y = v_y * vel_norm_goal / vel_norm
                        v_z = v_z * vel_norm_goal / vel_norm
                    if w_norm > w_norm_goal:
                        w = w * w_norm_goal / w_norm
                    # apply the desired velocity:
                    self.pid_vel_x.setpoint = v_x
                    self.pid_vel_y.setpoint = v_y
                    self.pid_vel_z.setpoint = v_z
                    self.pid_w_z.setpoint = w
                    self.pid_vel_x.set_auto_mode(True)
                    self.pid_vel_y.set_auto_mode(True)
                    if v_z != 0:
                        self.pid_vel_z.set_auto_mode(True)
                    else:
                        self.pid_vel_z.set_auto_mode(False)
                    if w != 0:
                        self.pid_w_z.set_auto_mode(True)
                    else:
                        self.pid_w_z.set_auto_mode(False)
                    x_target = self.pid_vel_x(vel_x_df)
                    y_target = self.pid_vel_y(vel_y_df)
                    z_target = self.pid_vel_z(vel_z_df) if v_z != 0 else 0
                    w_target = self.pid_w_z(yaw_rate) if w != 0 else 0
                    self.CMDS['pitch'] = DEFAULT_PITCH + x_target
                    self.CMDS['roll'] = DEFAULT_ROLL + y_target
                    self.CMDS['throttle'] = self.CMDS['throttle'] + z_target
                    self.CMDS['yaw'] = DEFAULT_YAW + w_target
            elif self.current_flight_command[0] in ("PDF", "PDZ"):  # position command drone frame
                # command is ["PDF", x, y, z, yaw, vel_norm_goal, w_norm_goal, duration]
                # we convert this into a command in world frame (it will be applied in the next iteration)
                current_flight_command = list(self.current_flight_command)
                current_flight_command[0] = "PWF"
                x_goal_df = current_flight_command[1]
                y_goal_df = current_flight_command[2]
                x_goal_wf = x_goal_df * cos - y_goal_df * sin + pos_x_wf
                y_goal_wf = y_goal_df * cos + x_goal_df * sin + pos_y_wf
                yaw_goal_df = current_flight_command[4]
                yaw_goal_wf = (yaw + yaw_goal_df) % (2 * np.pi) if yaw_goal_df is not None else None
                current_flight_command[1] = x_goal_wf
                current_flight_command[2] = y_goal_wf
                current_flight_command[4] = yaw_goal_wf
                if self.current_flight_command[0] == "PDZ" and current_flight_command[3] is not None:
                    current_flight_command[3] = pos_z_wf
                self.current_flight_command = tuple(current_flight_command)

        if self.trace_logs:
            now = time.time() - self.start_time
            self.x_tracer.write_line(f"{now};{pos_x_wf};{vel_x_wf};{vel_x_df};{self.pid_vel_x.setpoint};{int(self.pid_vel_x._auto_mode)}")
            self.y_tracer.write_line(f"{now};{pos_y_wf};{vel_y_wf};{vel_y_df};{self.pid_vel_y.setpoint};{int(self.pid_vel_y._auto_mode)}")
            self.z_tracer.write_line(f"{now};{pos_z_wf};{vel_z_wf};{vel_z_df};{self.pid_vel_z.setpoint};{int(self.pid_vel_z._auto_mode)}")
            self.w_tracer.write_line(f"{now};{yaw};{yaw_rate};{yaw_rate};{self.pid_w_z.setpoint};{int(self.pid_w_z._auto_mode)}")

    def _check_batt_voltage(self):
        if self.min_voltage < self.voltage <= self.warn_voltage:
            self.__batt_state = BATT_WARNING
        elif self.voltage <= self.min_voltage:
            self.__batt_state = BATT_TOO_LOW
        elif self.voltage >= self.max_voltage:
            self.__batt_state = BATT_TOO_HIGH
        else:
            self.__batt_state = BATT_OK

    def _batt_handler(self, board):
        """
        This overrides flight and UDP when battery voltage is out of range
        """
        if self.__batt_state != BATT_OK:
            if self.__batt_state == BATT_TOO_LOW:
                self.emergency = True
            if self.sender_initialized and time.time() - self.last_bat_tick >= BATT_UDP_MSG_TIME:
                self.udp_int.send(pkl.dumps(("BBT", (self.voltage, ))))
                self.last_bat_tick = time.time()

    def _emergency_handler(self, board):
        """
        This overrides all commands when an emergency occurs
        """
        if self.emergency:
            self.CMDS['roll'] = DEFAULT_ROLL
            self.CMDS['pitch'] = DEFAULT_PITCH
            self.CMDS['throttle'] = LAND
            self.CMDS['yaw'] = DEFAULT_YAW
            board.fast_read_altitude()
            if board.SENSOR_DATA['altitude'] <= 0.1:
                self.CMDS['aux1'] = DISARMED
                self.CMDS['throttle'] = DEFAULT_THROTTLE
                self._flight_origin = None  # reset flight origin after an emergency
                self.emergency = False

    def _controller(self, screen, profiling_duration=-1):
        # print doesn't work with curses, use addstr instead
        profile = False

        try:
            if self.print_screen:
                try_addstr(screen, 15, 0, "Connecting to the FC...")

            with MSPy(device=self.device_str, loglevel='WARNING', baudrate=115200, timeout=0) as board:
                if board == 1:  # an error occurred...
                    return 1
                else:
                    self.board = board
                    if self.pose_estimator is not None:
                        self.pose_estimator.get_fc_estimate = self._read_board.__get__(self, PoseEstimator)

                if self.print_screen:
                    try_addstr(screen, 15, 0, "Connecting to the FC... connected!")
                    screen.clrtoeol()
                    screen.move(1, 0)

                average_cycle = deque([0.0] * NO_OF_CYCLES_AVERAGE_GUI_TIME)

                # It's necessary to send some messages or the RX failsafe will be active
                # and it will not be possible to arm.
                command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                                'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                                'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

                if board.INAV:
                    command_list.append('MSP2_INAV_ANALOG')
                    command_list.append('MSP_VOLTAGE_METER_CONFIG')

                for msg in command_list:
                    if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                        data_handler = board.receive_msg()
                        board.process_recv_data(data_handler)
                if board.INAV:
                    cell_count = board.BATTERY_STATE['cellCount']
                else:
                    cell_count = 0  # MSP2_INAV_ANALOG is necessary
                self.min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage'] * cell_count
                self.warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage'] * cell_count
                self.max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage'] * cell_count

                if self.print_screen:
                    try_addstr(screen, 15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
                    screen.clrtoeol()
                    try_addstr(screen, 15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
                    try_addstr(screen, 16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
                    try_addstr(screen, 16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
                    try_addstr(screen, 17, 0, "boardName: {}".format(board.CONFIG['boardName']))
                    try_addstr(screen, 17, 50, "name: {}".format(board.CONFIG['name']))

                slow_msgs = cycle(['MSP_ANALOG', 'MSP_MOTOR', 'MSP_STATUS_EX', 'MSP_RC'])

                # send the "drone ready" command to the udp controller:
                self.voltage = board.ANALOG['voltage']
                self._check_batt_voltage()

                cursor_msg = ""

                if profiling_duration >= 0:
                    from pyinstrument import Profiler
                    profile = True
                    pro = Profiler()
                    stop_profiling_time = time.time() + profiling_duration
                    pro.start()

                last_loop_time = last_slow_msg_time = last_cycle_time = time.time()
                while True:
                    start_time = time.time()

                    #
                    # checking connection result  (NO DELAYS) ------------------
                    #
                    if self.udp_int is None and self.network:
                        with self._lock_connect:
                            if self._udp_int is not None:
                                self.drone_hostname = self._drone_hostname
                                self.drone_ip = self._drone_ip
                                self.drone_port = self._drone_port
                                self.udp_int = self._udp_int
                                self.tcp_video_int = self._tcp_video_int

                    #
                    # Gamepad manager  (NO DELAYS) -----------------------------
                    # UDP commands are ignored when a gamepad is connected
                    # Gamepad commands are overridden by key presses
                    #

                    self.CMDS, self.current_flight_command, emergency, mode = self.gamepad_manager.get(self.CMDS, self.current_flight_command)
                    if emergency and not self.emergency:
                        self.emergency = True
                    override = mode == 1
                    gamepad = mode != 0

                    #
                    # UDP recv non-blocking  (NO DELAYS) -----------------------
                    # For safety, UDP commands are overridden by key presses
                    #
                    if self.udp_int:
                        udp_cmds = self.udp_int.recv_nonblocking()
                        if len(udp_cmds) > 0:
                            for cmd in udp_cmds:
                                self._udp_commands_handler(pkl.loads(cmd))
                        if not override:  # override the flight controller if valid PS4
                            self._flight(board, screen)
                        if self.obs_loop_time is not None:
                            tick = time.time()
                            if tick - self.last_obs_tick >= self.obs_loop_time and self.sender_initialized:
                                self.last_obs_tick = tick
                                self._i_obs += 1
                                self.udp_int.send(pkl.dumps(("OBS", self._i_obs, self.telemetry, gamepad, self.voltage, self.debug_flags)))
                    #
                    # end of UDP recv non-blocking -----------------------------
                    #

                    self._batt_handler(board)
                    self._emergency_handler(board)

                    if self.print_screen:
                        char = screen.getch()  # get keypress
                        curses.flushinp()  # flushes buffer
                        #
                        # KEYS (NO DELAYS) -----------------------------------------
                        #
                        if char != -1:
                            self.key_cmd_in_progress = True
                            self.last_key_tick = time.time()

                            if char == QUIT_CHR:
                                break

                            elif char == DISARM_CHR:
                                cursor_msg = 'Disarming...'
                                self.CMDS['aux1'] = DISARMED

                            elif char == REBOOT_CHR:
                                if self.print_screen:
                                    try_addstr(screen, 3, 0, 'Rebooting...')
                                    screen.clrtoeol()
                                board.reboot()
                                time.sleep(0.5)
                                break

                            elif char == ARM_CHR:
                                cursor_msg = 'Arming...'
                                self.CMDS['aux1'] = ARMED

                            elif char == SWITCH_MODE_CHR:
                                if self.CMDS['aux2'] <= 1300:
                                    cursor_msg = 'NAV ALTHOLD Mode...'
                                    self.CMDS['aux2'] = 1500
                                elif 1700 > self.CMDS['aux2'] > 1300:
                                    cursor_msg = 'NAV POSHOLD Mode...'
                                    self.CMDS['aux2'] = 1800
                                elif self.CMDS['aux2'] >= 1650:
                                    cursor_msg = 'Angle Mode...'
                                    self.CMDS['aux2'] = 1000

                            elif char == RIGHT_CHR:
                                self.CMDS['roll'] = KEY_P_ROLL
                                cursor_msg = 'roll(+):{}'.format(self.CMDS['roll'])

                            elif char == LEFT_CHR:
                                self.CMDS['roll'] = KEY_N_ROLL
                                cursor_msg = 'roll(-):{}'.format(self.CMDS['roll'])

                            elif char == RIGHTYAW_CHR:
                                self.CMDS['yaw'] = KEY_P_YAW
                                cursor_msg = 'yaw(+):{}'.format(self.CMDS['yaw'])

                            elif char == LEFTYAW_CHR:
                                self.CMDS['yaw'] = KEY_N_YAW
                                cursor_msg = 'yaw(-):{}'.format(self.CMDS['yaw'])

                            elif char == FORWARD_CHR:
                                self.CMDS['pitch'] = KEY_P_PITCH
                                cursor_msg = 'pitch(+):{}'.format(self.CMDS['pitch'])

                            elif char == BACKWARD_CHR:
                                self.CMDS['pitch'] = KEY_N_PITCH
                                cursor_msg = 'pitch(-):{}'.format(self.CMDS['pitch'])

                            elif char == UPWARD_CHR:
                                self.CMDS['throttle'] = self.CMDS['throttle'] + KEY_P_THROTTLE if self.CMDS['throttle'] + KEY_P_THROTTLE <= MAX_CMD_THROTTLE else self.CMDS['throttle']
                                cursor_msg = 'throttle(+):{}'.format(self.CMDS['throttle'])

                            elif char == DOWNWARD_CHR:
                                self.CMDS['throttle'] = self.CMDS['throttle'] + KEY_N_THROTTLE if self.CMDS['throttle'] + KEY_N_THROTTLE >= MIN_CMD_THROTTLE else self.CMDS['throttle']
                                cursor_msg = 'throttle(-):{}'.format(self.CMDS['throttle'])

                            elif char == TAKEOFF_CHR:
                                self.CMDS['throttle'] = KEY_TAKEOFF
                                cursor_msg = 'takeoff throttle:{}'.format(self.CMDS['throttle'])

                            elif char == LAND_CHR:
                                self.CMDS['throttle'] = KEY_LAND
                                cursor_msg = 'land throttle:{}'.format(self.CMDS['throttle'])

                            elif PAUSE_CHR:
                                self.CMDS['roll'] = DEFAULT_ROLL
                                self.CMDS['pitch'] = DEFAULT_PITCH
                                self.CMDS['yaw'] = DEFAULT_YAW

                        elif self.key_cmd_in_progress:  # default behavior
                            if time.time() - self.last_key_tick >= KEY_TIMEOUT:
                                self.key_cmd_in_progress = False
                                self.CMDS['roll'] = DEFAULT_ROLL
                                self.CMDS['pitch'] = DEFAULT_PITCH
                                self.CMDS['yaw'] = DEFAULT_YAW
                        #
                        # End of KEYS ----------------------------------------------
                        #

                    #
                    # CLIP RC VALUES -------------------------------------------
                    #
                    self.CMDS['roll'] = clip(self.CMDS['roll'], MIN_CMD_ROLL, MAX_CMD_ROLL)
                    self.CMDS['pitch'] = clip(self.CMDS['pitch'], MIN_CMD_PITCH, MAX_CMD_PITCH)
                    self.CMDS['yaw'] = clip(self.CMDS['yaw'], MIN_CMD_YAW, MAX_CMD_YAW)
                    self.CMDS['throttle'] = clip(self.CMDS['throttle'], MIN_CMD_THROTTLE, MAX_CMD_THROTTLE)
                    #
                    # END CLIP RC VALUES ---------------------------------------
                    #

                    #
                    # RESET ON ARMING ------------------------------------------
                    #
                    if self.CMDS['aux1'] == ARMED:
                        if not self._armed:
                            self._armed = True
                            self.current_flight_command = None
                            self._reset_pids()
                            self.CMDS['roll'] = DEFAULT_ROLL
                            self.CMDS['pitch'] = DEFAULT_PITCH
                            self.CMDS['yaw'] = DEFAULT_YAW
                            self.CMDS['throttle'] = DEFAULT_THROTTLE
                    else:
                        self._armed = False
                        self.current_flight_command = None
                        self._reset_pids()
                        self.CMDS['roll'] = DEFAULT_ROLL
                        self.CMDS['pitch'] = DEFAULT_PITCH
                        self.CMDS['yaw'] = DEFAULT_YAW
                        self.CMDS['throttle'] = DEFAULT_THROTTLE
                    #
                    # END RESET ON ARMING ---------------------------------------
                    #

                    #
                    # IMPORTANT MESSAGES (CTRL_LOOP_TIME based) ----------------
                    #
                    if (time.time() - last_loop_time) >= CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        # Send the RC channel values to the FC
                        if board.send_RAW_RC([int(self.CMDS[ki]) for ki in self.CMDS_ORDER]):
                            data_handler = board.receive_msg()
                            board.process_recv_data(data_handler)
                    #
                    # End of IMPORTANT MESSAGES --------------------------------
                    #

                    #
                    # SLOW MSG processing (user GUI and voltage) ---------------
                    #
                    if (time.time() - last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                        last_slow_msg_time = time.time()

                        next_msg = next(slow_msgs)  # circular list

                        if not self.print_screen:
                            if next_msg == 'MSP_ANALOG':
                                # Read info from the FC
                                if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                                    data_handler = board.receive_msg()
                                    board.process_recv_data(data_handler)
                                self.voltage = board.ANALOG['voltage']
                                self._check_batt_voltage()
                            elif next_msg == 'MSP_STATUS_EX':
                                # Read info from the FC
                                if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                                    data_handler = board.receive_msg()
                                    board.process_recv_data(data_handler)
                                self.debug_flags = board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])
                                if self.tcp_video_int is not None:
                                    cam_err, cam_exp, cam_trace = self.tcp_video_int.get_camera_error()
                                    if cam_err:
                                        self.debug_flags.append("CAMERA_ERROR")
                        else:  # print screen messages
                            # Read info from the FC
                            if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                                data_handler = board.receive_msg()
                                board.process_recv_data(data_handler)
                            if next_msg == 'MSP_ANALOG':
                                self.voltage = board.ANALOG['voltage']
                                self._check_batt_voltage()
                                voltage_msg = ""
                                if self.__batt_state == BATT_WARNING:
                                    voltage_msg = "LOW BATT WARNING"
                                elif self.__batt_state == BATT_TOO_LOW:
                                    voltage_msg = "ULTRA LOW BATT!!!"
                                elif self.__batt_state == BATT_TOO_HIGH:
                                    voltage_msg = "VOLTAGE TOO HIGH"

                                try_addstr(screen, 8, 0, "Battery Voltage: {:2.2f}V".format(self.voltage))
                                screen.clrtoeol()
                                try_addstr(screen, 8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                                screen.clrtoeol()

                            elif next_msg == 'MSP_STATUS_EX':
                                armed = board.bit_check(board.CONFIG['mode'], 0)
                                try_addstr(screen, 5, 0, "ARMED: {}".format(armed), curses.A_BOLD)
                                screen.clrtoeol()

                                self.debug_flags = board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])
                                if self.tcp_video_int is not None:
                                    cam_err, cam_exp, cam_trace = self.tcp_video_int.get_camera_error()
                                    if cam_err:
                                        self.debug_flags.append("CAMERA_ERROR")
                                        try_addstr(screen, 12, 0, f"CAMERA ERROR: {cam_exp}")
                                        screen.clrtoeol()
                                        # raise Exception(cam_trace)
                                try_addstr(screen, 5, 50, "armingDisableFlags: {}".format(self.debug_flags))
                                screen.clrtoeol()

                                try_addstr(screen, 6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                                screen.clrtoeol()
                                try_addstr(screen, 6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                                screen.clrtoeol()

                                try_addstr(screen, 7, 0, "mode: {}".format(board.CONFIG['mode']))
                                screen.clrtoeol()

                                try_addstr(screen, 7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                                screen.clrtoeol()

                            elif next_msg == 'MSP_MOTOR':
                                try_addstr(screen, 9, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                                screen.clrtoeol()

                            elif next_msg == 'MSP_RC':
                                try_addstr(screen, 10, 0, "RC Channels Values: {}".format(board.RC['channels']))
                                screen.clrtoeol()

                            _s1 = sum(average_cycle)
                            _s2 = len(average_cycle)
                            str_cycletime = "NaN" if _s1 == 0 or _s2 == 0 else \
                                f"GUI cycleTime: {last_cycle_time * 1000:2.2f}ms (average {1 / (_s1 / _s2):2.2f}Hz), barometer: {self._d1 * 1000:2.2f}ms, gps: {self._d2 * 1000:2.2f}ms, compass: {self._d3 * 1000:2.2f}ms"
                            try_addstr(screen, 11, 0, str_cycletime)
                            screen.clrtoeol()

                            try_addstr(screen, 3, 0, cursor_msg)
                            screen.clrtoeol()
                    #
                    # end of SLOW MSG ------------------------------------------
                    #

                    # This slows the loop down to CTRL_LOOP_TIME:
                    end_time = time.time()
                    last_cycle_time = end_time - start_time
                    if last_cycle_time < CTRL_LOOP_TIME:
                        time.sleep(CTRL_LOOP_TIME - last_cycle_time)

                    average_cycle.append(last_cycle_time)
                    average_cycle.popleft()

                    if profile:
                        if time.time() >= stop_profiling_time:
                            pro.stop()
                            return pro.output_text()

        finally:
            self.board = None
            if self.print_screen:
                try_addstr(screen, 5, 0, "Disconnected from the FC!")
                screen.clrtoeol()
            print("Bye!")


def run_controller(print_screen=True, trace_logs=False):
    cc = CogniflyController(print_screen=print_screen, trace_logs=trace_logs)
    cc.run_curses()


if __name__ == "__main__":
    run_controller()
