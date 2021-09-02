from copy import deepcopy
from threading import Lock, Thread
import socket
import pickle as pkl
import logging
import time
import numpy as np

from cognifly.utils.udp_interface import UDPInterface
from cognifly.utils.tcp_video_interface import TCPVideoInterface
from cognifly.utils.ip_tools import extract_ip
from cognifly.utils.functions import clip, get_angle_sequence_rad

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)


# Constants for the school easy API

EASY_API_SPEED = 0.2  # (m/s)
EASY_API_YAW_RATE = 0.5  # (rad/s)
EASY_API_ADDITIONAL_DURATION = 10.0  # (s)
EASY_API_TAKEOFF_ALTITUDE = 0.5  # should be roughly the same as default takeoff altitude (m)
EASY_API_MAX_ALTITUDE = 0.9
EASY_API_MIN_ALTITUDE = 0.35


# Remote controller class

class Cognifly:
    """
    Remote UDP controller meant to communicate with a drone that executes cognifly_controller.py
    Each command is sent via UDP
    The command is sent along an identifier that is sent back by the drone as an aknowledgement
    If the acknowledgement of the last sent command is not received after wait_ack_duration seconds, the last command is sent again
    Any command received by the drone overrides the previous command instantaneously
    """
    def __init__(self,
                 drone_hostname,
                 local_hostname=None,
                 send_port=8988,
                 recv_port=8989,
                 recv_port_video=8990,
                 wait_for_first_obs=True,
                 wait_for_first_obs_sleep_duration=0.1,
                 wait_ack_duration=0.2,
                 gui=True):
        """
        Args:
            drone_hostname: string: name of the drone (can be an ip address).
            local_hostname: string: name of the local machine (can be an ip address). If None, this is retrieved automatically.
            send_port: int: udp_send_port for the local UDPInterface.
            recv_port: int: udp_recv_port for UDPInterface.
            wait_for_first_obs: int: If True, waits for a first observation from the drone on isntantiation.
            wait_for_first_obs_sleep_duration: float: used only if wait_for_first_obs is True
            wait_ack_duration: float: If > 0.0, the framework will wait this number of seconds for an acknowledgement before resending.
                If <= 0.0, the resending feature is disabled.
            gui: bool: if True, a gui window will pop when the object is created (emergency disarm, battery, streaming).
        """
        self.gui = gui
        self.drone_hostname = drone_hostname
        self.send_port = send_port
        self.recv_port = recv_port
        self.recv_port_video = recv_port_video
        self.wait_for_first_obs = wait_for_first_obs
        self.wait_for_first_obs_sleep_duration = wait_for_first_obs_sleep_duration

        self.udp_int = UDPInterface()
        self.tcp_video_int = TCPVideoInterface()

        self.local_hostname = socket.gethostname() if local_hostname is None else local_hostname
        self.local_ip = socket.gethostbyname(self.local_hostname) if local_hostname is not None else extract_ip()
        logger.info(f"local hostname: {self.local_hostname} with IP: {self.local_ip}")
        self.drone_hostname = drone_hostname
        self.drone_ip = socket.gethostbyname(drone_hostname)
        logger.info(f"drone hostname: {self.drone_hostname} with IP: {self.drone_ip}")

        self.udp_int.init_sender(self.drone_ip, self.send_port)
        self.udp_int.init_receiver(self.local_ip, self.recv_port)

        self.wait_ack_duration = wait_ack_duration
        self.last_im_id = -1


        self.easy_api_cur_z = 0.0
        self.time_takeoff = time.time()

        self._lock = Lock()  # used to update the observation and also to resend
        self.__obs = None
        self.__last_sent_id = 0
        self.__last_received_id = 0
        self.__last_timestamp = time.time()
        self.__last_sent = None
        self.__wait_ack_reset = True
        self.__last_i_obs = 0
        self.__wait_done = False

        self._display_lock = Lock()
        self.__display = False

        self._batt_lock = Lock()
        self.__batt_str = "OK"

        self._listener_thread = Thread(target=self.__listener_thread)
        self._listener_thread.setDaemon(True)  # thread will be terminated at exit
        self._listener_thread.start()
        if self.wait_ack_duration > 0:
            self._resender_thread = Thread(target=self.__resender_thread)
            self._resender_thread.setDaemon(True)  # thread will be terminated at exit
            self._resender_thread.start()
        else:
            self._resender_thread = None

        self.reset()

        if self.gui:
            self._gui_thread = Thread(target=self.__gui_thread, args=(self.drone_hostname, ))
            self._gui_thread.setDaemon(True)  # thread will be terminated at exit
            self._gui_thread.start()
        else:
            self._gui_thread = None

    def __gui_thread(self, drone_name):
        """
        Thread in charge of managing the Graphical User Interface
        """
        import PySimpleGUI as sg
        import cv2
        sg.theme('BluePurple')
        layout = [[sg.Text('Battery:'), sg.Text('OK', size=(15, 1), key='-batt-')],
                  [sg.Button('DISARM')],
                  [sg.Image(filename='', key='-image-')]]
        window = sg.Window(drone_name,
                           layout,
                           keep_on_top=True,
                           enable_close_attempted_event=True)
        image_elem = window['-image-']
        batt_elem = window['-batt-']
        try:
            while True:  # Event Loop
                with self._display_lock:
                    # check whether the display is on:
                    if self.__display:
                        event, values = window.read(0)
                        with self.tcp_video_int.condition_in:
                            self.tcp_video_int.condition_in.wait()  # wait for new frame
                            frame, _ = self.tcp_video_int.get_last_image()
                        if frame is not None:  # should not be None but let's be cautious
                            imgbytes = cv2.imencode('.png', frame)[1].tobytes()  # ditto
                            image_elem.update(data=imgbytes)
                    else:
                        event, values = window.read(200)  # 5 Hz to check change of self.__display
                        image_elem.update(data=b'')
                if event == 'DISARM':
                    self.disarm()  # thread-safe because the whole self.send procedure uses self._lock
                if event == sg.WINDOW_CLOSE_ATTEMPTED_EVENT and sg.popup_yes_no('Do you really want to exit?', keep_on_top=True) == 'Yes':
                    break
                with self._batt_lock:
                    batt_elem.update(self.__batt_str)
        finally:
            window.close()

    def __listener_thread(self):
        """
        Thread in charge of receiving UDP messages.
        """
        while True:
            mes = self.udp_int.recv()
            for mb in mes:
                m = pkl.loads(mb)
                if m[0] == "ACK":  # aknowledgement type
                    self._lock.acquire()
                    if m[1] > self.__last_received_id:
                        self.__last_received_id = m[1]  # identifier
                    self._lock.release()
                elif m[0] == "OBS":  # observation type
                    self._lock.acquire()
                    i_obs = m[1]
                    if i_obs > self.__last_i_obs:  # drop outdated observations
                        self.__obs = m[2]  # observation
                        self.__last_i_obs = i_obs
                    self._lock.release()
                elif m[0] == "RES":  # acknowledgement for the reset command
                    self._lock.acquire()
                    self.__wait_ack_reset = False
                    self._lock.release()
                elif m[0] == "DON":  # 'done' signal from position control
                    self._lock.acquire()
                    if self.__last_sent_id == m[1]:
                        self.__wait_done = False
                    self._lock.release()
                elif m[0] == "BBT":  # bad battery state
                    with self._batt_lock:
                        self.__batt_str = "LOW!"

    def __resender_thread(self):
        """
        Thread in charge of resending the last UDP message when it has not been acknowledged by the drone.
        """
        while True:
            time.sleep(self.wait_ack_duration)
            self._lock.acquire()
            if self.__last_received_id < self.__last_sent_id:
                t = time.time()
                if t - self.__last_timestamp > self.wait_ack_duration:
                    self.udp_int.send(self.__last_sent)  # resend (with the same identifier as before)
                    self.__last_timestamp = t
            self._lock.release()

    def send(self, msg_type, msg=None):
        """
        Send an UDP message to the drone.
        Args:
            msg_type: string in ["RES", ACT"].
            msg: tuple of the form (str:command, ...:args).
        """
        self._lock.acquire()
        self.__last_sent_id += 1
        self.__last_timestamp = time.time()
        data = (msg_type, self.__last_sent_id, msg) if msg is not None else (msg_type, self.__last_sent_id)
        data_s = pkl.dumps(data)
        self.__last_sent = data_s
        self.udp_int.send(data_s)
        self._lock.release()

    def sleep_until_done(self, max_duration, granularity=0.05):
        """
        Sleeps until either max_duration is elapsed or a 'done" signal is received from the position control API.
        """
        timeout_t = time.time() + max_duration
        while True:
            time.sleep(granularity)
            self._lock.acquire()
            c = self.__wait_done
            self._lock.release()
            t = time.time()
            if not c or t >= timeout_t:
                if c:
                    self._lock.acquire()
                    self.__wait_done = False
                    self._lock.release()
                break

    # ==================================================================================================================

    # Pro API
    # Use this except for school purpose:

    def reset(self, sleep_duration=0.1):
        """
        Resets the drone and waits for reset acknowlegement.
        Args:
            sleep_duration: float: the reset command waits for at least this amount of time
        """
        self.send(msg_type="RES", msg=(self.local_ip, self.recv_port))
        # wait for the RES message to return
        c = True
        logger.info("Waiting for reset acknowlegement...")
        while c:
            time.sleep(sleep_duration)
            self._lock.acquire()
            c = self.__wait_ack_reset
            if self.__wait_ack_reset:
                self.__wait_ack_reset = True  # for next reset
            self._lock.release()
        logger.info("Reset acknowlegement received.")
        self._lock.acquire()
        self.__obs = None
        self.__last_sent_id = 0
        self.__last_received_id = 0
        self.__last_timestamp = time.time()
        self.__last_sent = None
        self._lock.release()
        if self.wait_for_first_obs:
            logger.info("waiting for a first observation from the drone")
            while True:
                time.sleep(self.wait_for_first_obs_sleep_duration)
                self._lock.acquire()
                obs = deepcopy(self.__obs)
                self._lock.release()
                if obs is not None:
                    break
            logger.info("observation initialized")

    def arm(self):
        """
        Arms the drone.
        """
        self.send(msg_type="ACT", msg=("ARM",))

    def disarm(self):
        """
        Disarms the drone.
        """
        self.send(msg_type="ACT", msg=("DISARM",))

    def takeoff_nonblocking(self, altitude=None):
        """
        The drone takes off.
        """
        self.send(msg_type="ACT", msg=("TAKEOFF", altitude))
        self.time_takeoff = time.time()

    def land_nonblocking(self):
        """
        The drone lands.
        """
        self.send(msg_type="ACT", msg=("LAND",))

    def set_velocity_nonblocking(self, v_x=0.0, v_y=0.0, v_z=0.0, w=0.0, duration=1.0, drone_frame=True):
        """
        Sets the drone velocity.
        The drone needs to be armed.
        Args:
            v_x: float (optional): velocity target on the frontward axis (m/s); tested range: 0.1 - 0.5
            v_y: float (optional): velocity target on the rightward axis (m/s); tested range: 0.1 - 0.5
            v_z: float (optional): velocity target on the upward axis (m/s); tested range: 0.05 - 0.1
            w: float (optional): yaw rate on the upward axis (rad/s)); tested range: 0.05 - 0.5
            duration: float (optional): maximum max_duration of the command (s)
            drone_frame: bool (optional): whether to use the drone frame (True) or the world frame (False)
        """
        frame_str = "VDF" if drone_frame else "VWF"
        self.send(msg_type="ACT", msg=(frame_str, v_x, v_y, v_z, w, duration))

    def set_position_nonblocking(self, x, y, z, yaw=None, max_velocity=0.5, max_yaw_rate=0.5, max_duration=10.0, relative=False):
        """
        Sets a position target for the drone.
        If relative is False (default), the target is relative to the world axis.
        If relative is True, the target is relative to the drone frame.
        Note: z is always in the world frame.
        The drone needs to be armed.
        Args:
            x: float: x target (m)
            y: float: y target (m)
            z: float: z target (m); tested range: 0.3 - 0.9
            yaw: float (optional): yaw target (rad); if None, the yaw is not changed
            max_velocity: float (optional): maximum velocity used to go to position (m/s)
            max_yaw_rate: float (optional): maximum yaw rate used to go to yaw (rad/s)
            max_duration: float (optional): maximum duration of the command (s)
            relative: bool (optional): whether to use the drone frame (True) or the world frame (False, default)
        """
        if relative:
            frame_str = "PDF"
        else:
            frame_str = "PWF"
        self._lock.acquire()
        self.__wait_done = True
        self._lock.release()
        self.send(msg_type="ACT", msg=(frame_str, x, y, z, yaw, max_velocity, max_yaw_rate, max_duration))

    # ==================================================================================================================

    # High-level API for school purpose
    # (don't use along the pro API, otherwise weird things will happen with altitude)
    # (sleeps after calls, uses cm instead of m, and uses degrees instead of rad):

    def takeoff(self, altitude=None, alt_duration=10.0):
        """
        Arms the drone and takes off
        Args:
            altitude: float (optional): target altitude (cm)
            alt_duration: float (optional): additional max duration to reach the target altitude
        """
        altitude = altitude / 100.0 if altitude is not None else EASY_API_TAKEOFF_ALTITUDE
        self.reset()
        self.arm()
        time.sleep(1.0)
        # the takeoff altitude might depend on the battery with this and Z will only be defined properly later:
        self.takeoff_nonblocking()
        time.sleep(10.0)
        self.easy_api_cur_z = altitude
        # this works (Z properly defined) but when done from the ground it is a bit violent and not very stable:
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=0.0,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=alt_duration,
                                      relative=False)
        self.sleep_until_done(alt_duration)

    def land(self, sleep_duration=5.0):
        """
        Lands and disarms the drone
        """
        self.easy_api_cur_z = 0.0
        self.land_nonblocking()
        time.sleep(sleep_duration)
        self.disarm()

    def forward(self, val_cm):
        """
        Goes forward by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=val_m, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def backward(self, val_cm):
        """
        Goes backward by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=-val_m, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def right(self, val_cm):
        """
        Goes right by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=val_m, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def left(self, val_cm):
        """
        Goes left by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=-val_m, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def up(self, val_cm):
        """
        Goes up by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.easy_api_cur_z += val_m
        self.easy_api_cur_z = clip(self.easy_api_cur_z, EASY_API_MIN_ALTITUDE, EASY_API_MAX_ALTITUDE)
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def down(self, val_cm):
        """
        Goes down by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.easy_api_cur_z -= val_m
        self.easy_api_cur_z = clip(self.easy_api_cur_z, EASY_API_MIN_ALTITUDE, EASY_API_MAX_ALTITUDE)
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def cw(self, val_deg):
        """
        Rotates clockwise by number of degrees (range 0 - 180).
        Note: the drone takes the shortest path, direction is not guaranteed for angles close to 180.
        """
        # val_rad = max(val_deg * np.pi / 180.0, 0.0)
        # duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        # seq_angles = get_angle_sequence_rad(val_rad)
        # seq = []
        # for sa in seq_angles:
        #     seq.append([0, 0, self.easy_api_cur_z, sa])
        # self.position_sequence(sequence=seq, max_duration=duration, relative=True)

        val_rad = np.pi if val_deg == 180 else (val_deg % 180) * np.pi / 180.0
        duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=val_rad,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def ccw(self, val_deg):
        """
        Rotates counter-clockwise by number of degrees (range 0 - 180).
        Note: the drone takes the shortest path, direction is not guaranteed for angles close to 180.
        """
        # val_rad = max(val_deg * np.pi / 180.0, 0.0)
        # duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        # val_rad = - val_rad
        # seq_angles = get_angle_sequence_rad(val_rad)
        # seq = []
        # for sa in seq_angles:
        #     seq.append([0, 0, self.easy_api_cur_z, sa])
        # self.position_sequence(sequence=seq, max_duration=duration, relative=True)

        val_rad = np.pi if val_deg == 180 else (val_deg % 180) * np.pi / 180.0
        duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=-val_rad,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        self.sleep_until_done(duration)

    def go(self, x, y, z, yaw=None, max_duration=10.0):
        """
        Goes to the requested position (cm) and yaw (deg) targets.
        If yaw is None, no yaw target is set.
        """
        x = x / 100.0
        y = y / 100.0
        z = z / 100.0
        y = y * np.pi / 180.0
        self.easy_api_cur_z = clip(z, EASY_API_MIN_ALTITUDE, EASY_API_MAX_ALTITUDE)
        self.set_position_nonblocking(x=x, y=y, z=self.easy_api_cur_z, yaw=yaw,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=max_duration,
                                      relative=False)
        self.sleep_until_done(max_duration)

    def position_sequence(self, sequence, max_duration=60.0, relative=False, speed=None, yaw_rate=None):
        """
        The drone follows a roadmap defined by a sequence of targets.
        Args:
            sequence: a sequence of sequence-like elements, each of length 3 or 4 (mixing 3 and 4 is possible).
                If the length of an element is 3, it is interpreted as (x, y, z) (cm).
                If the length of an element is 4, it is interpreted as (x, y, z, yaw) (cm and deg).
            max_duration: float: maximum duration of the whole command.
            relative: bool: whether x, y and yaw have to be interpreted in the drone frame (True)
                or in the world frame (False, default).
                Note: z is always in the world frame.
            speed: float: if not None, target speed in cm/s, else the default is used.
            yaw_rate: float: if not None, target yaw rate in deg/s, else the default is used.
        """
        if speed is not None:
            speed = speed / 100.0  # convert to m/s
            speed = max(speed, 0.0)
        if yaw_rate is not None:
            yaw_rate = yaw_rate * np.pi / 180.0  # convert to rad/s
            yaw_rate = max(yaw_rate, 0.0)
        remaining_duration = max_duration
        t_start = time.time()
        for elt in sequence:
            if 3 <= len(elt) <= 4 and remaining_duration > 0:
                x = elt[0] / 100.0  # convert to cm
                y = elt[1] / 100.0  # convert to cm
                z = elt[2] / 100.0  # convert to cm
                yaw = elt[3] * np.pi / 180.0 if len(elt) == 4 else None  # convert to rad

                self.easy_api_cur_z = clip(z, EASY_API_MIN_ALTITUDE, EASY_API_MAX_ALTITUDE)
                self.set_position_nonblocking(x=x, y=y, z=self.easy_api_cur_z, yaw=yaw,
                                              max_velocity=EASY_API_SPEED if speed is None else speed,
                                              max_yaw_rate=EASY_API_YAW_RATE if yaw_rate is None else yaw_rate,
                                              max_duration=remaining_duration,
                                              relative=relative)
                self.sleep_until_done(remaining_duration)
                elapsed_time = time.time() - t_start
                remaining_duration -= elapsed_time

    def curve(self, x1, y1, z1, x2, y2, z2, speed):
        """
        2-positions roadmap.
        This is an alias for position_sequence(sequence=[[x1, y1, z1],[x2, y2, z2]], speed=speed).
        """
        self.position_sequence(sequence=[[x1, y1, z1],[x2, y2, z2]], speed=speed)

    # ==================================================================================================================

    # Methods common to both APIs:

    def streamon(self, resolution="VGA", fps=24, display=False, wait_first_frame=True):
        """
        Starts camera streaming.
        CAUTION: This will slow the frequency of the onboard controller down and may make the drone unstable!
        In particular, we recommend not having the stream on during takeoff when using the school API.
        The highest resolution and fps are, the worst the influence on the onboard controller.
        The image transfers happens over TCP, which comes with a noticeable delay.
        Args:
            resolution: Tuple(float: height, float: width): resolution of the captured images.
            fps: integer: maximum framerate (may not be attained),
            display: boolean: whether to display the stream in an OpenCV window,
                mostly for debugging purpose: this may output warnings or not work depending on opencv installation,
                in particular, the window is created and updated in the receiver thread.
            wait_first_frame: boolean: whether to sleep until the first frame is available.
        """
        self.tcp_video_int.start_receiver(self.recv_port_video, display)
        time.sleep(1.0) # sleep a bit so the server starts before the drone tries to connect
        self.send(msg_type="ST1", msg=(self.local_ip, self.recv_port_video, resolution, fps))
        if wait_first_frame:
            _ = self.get_frame(wait_new_frame=True)
        self._display_lock.acquire()
        self.__display = display
        self._display_lock.release()

    def stream(self, resolution="VGA", fps=24):
        """
        Alias for streamon(resolution, fps=5, display=True)
        """
        self.streamon(resolution, fps, display=True)

    def get_frame(self, wait_new_frame=True, timeout=10.0, sleep_between_trials=0.05):
        """
        Retrieves a single frame from the stream.
        Args:
            wait_new_frame: boolean: if True, two consecutive calls to the fonction will not return identical frames.
            timeout: after this duration without retrieving a frame, a TimeoutException will be raised.
            sleep_between_trials: float: will sleep this amount of time between two consecutive trials or retrieval.
        """
        t_start = time.time()
        im = None
        im_id = self.last_im_id if wait_new_frame else -1
        cond = True
        while cond:
            im, im_id = self.tcp_video_int.get_last_image()
            cond1 = im is None and im_id <= self.last_im_id
            cond = cond1 and time.time() - t_start < timeout
            if cond:
                time.sleep(sleep_between_trials)
        if cond1:
            raise TimeoutError("Could not retrieve frame from stream.")
        return im

    def streamoff(self):
        """
        Stops camera streaming
        """
        self._display_lock.acquire()
        self.__display = False
        self._display_lock.release()
        self.send(msg_type="ST0")
        time.sleep(1.0)
        self.tcp_video_int.stop_receiver()


    def get_time(self):
        """
        Gets the number of seconds since the last takeoff command was sent
        """
        return time.time() - self.time_takeoff

    def get_telemetry(self):
        """
        Gets a tuple that describe the state of the drone in the world frame.
        Angles are in radians.
        Returns:
            telemetry: Tuple: (battery_voltage: float,
                               x: float,
                               y: float,
                               z: float,
                               yaw: float,
                               vel_x: float,
                               vel_y: float,
                               vel_z: float,
                               yaw_rate: float,
                               health_flags: list of strings)
        """
        self._lock.acquire()
        obs = deepcopy(self.__obs)
        self._lock.release()
        return obs

    def get_battery(self):
        """
        Returns:
            battery_voltage: float: battery voltage, in V
        """
        telemetry = self.get_telemetry()
        return telemetry[0]

    def get_position(self):
        """
        Returns:
            x: float: in cm
            y: float: in cm
            z: float: in cm
        """
        telemetry = self.get_telemetry()
        return telemetry[1] * 100, telemetry[2] * 100, telemetry[3] * 100

    def get_yaw(self):
        """
        Returns:
            x: float: yaw, in deg
        """
        telemetry = self.get_telemetry()
        return telemetry[4] * 180.0 / np.pi

    def get_velocity(self):
        """
        Returns:
            v_x: float: in cm/s
            v_y: float: in cm/s
            v_z: float: in cm/s
        """
        telemetry = self.get_telemetry()
        return telemetry[5] * 100, telemetry[6] * 100, telemetry[7] * 100

    def get_health(self):
        """
        Returns:
            health_flags: list os strings: useful flags for debugging
        """
        telemetry = self.get_telemetry()
        return telemetry[9]

    def get_yaw_rate(self):
        """
        Returns:
            w: float: yaw rate in rad/s
        """
        telemetry = self.get_telemetry()
        return telemetry[8] * 180.0 / np.pi

    def get_tof(self):
        """
        Returns:
            h: float: altitude in cm
        """
        _, _, res = self.get_position()
        return res

    def get_height(self):
        """
        Alias for get_tof() as cognifly doesn't know its height from the starting point.
        """
        return self.get_tof()

    def get_speed(self):
        """
        Returns:
            speed: float: norm of the velocity, in cm/s
        """
        v_x, v_y, v_z = self.get_velocity()
        return np.linalg.norm([v_x, v_y, v_z]) * 100


if __name__ == '__main__':
    pass

    cf = Cognifly(drone_hostname="moderna.local")

    cf.streamon(fps=30)

    time.sleep(120.0)

    cf.streamoff()

    time.sleep(10.0)
