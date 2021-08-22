from copy import deepcopy
from threading import Lock, Thread
import socket
import pickle as pkl
import logging
import time
import numpy as np

from cognifly.utils.udp_interface import UDPInterface
from cognifly.utils.ip_tools import extract_ip
from cognifly.utils.functions import clip

logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)


# Constants for the school easy API

EASY_API_SPEED = 0.2  # (m/s)
EASY_API_YAW_RATE = 0.5  # (rad/s)
EASY_API_ADDITIONAL_DURATION = 5.0  # (s)
EASY_API_TAKEOFF_ALTITUDE = 0.5  # should be roughly the same as default takeoff altitude (m)
EASY_API_MAX_ALTITUDE = 0.9  # max altitude will be clipped at this value (m)
EASY_API_MIN_ALTITUDE = 0.35  # min altitude will be clipped at this value (m)


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
                 wait_for_first_obs=False,
                 wait_for_first_obs_sleep_duration=0.1,
                 wait_ack_duration=1.0):
        """
        Args:
            drone_hostname: string: name of the drone (can be an ip address).
            local_hostname: string: name of the local machine (can be an ip address). If None, this is retrieved automatically.
            send_port: int: udp_send_port for the local UDPInterface.
            recv_port: int: udp_recv_port for UDPInterface.
            wait_for_first_obs: int: If True, waits for a first observation from the drone on intantiation.
            wait_for_first_obs_sleep_duration: float: used only if wait_for_first_obs is True
            wait_ack_duration: float: If > 0.0, the framework will wait this number of seconds for an acknowledgement before resending.
                If <= 0.0, the resending feature is disabled.
        """
        self.drone_hostname = drone_hostname
        self.send_port = send_port
        self.recv_port = recv_port
        self.wait_for_first_obs = wait_for_first_obs
        self.wait_for_first_obs_sleep_duration = wait_for_first_obs_sleep_duration

        self.udp_int = UDPInterface()

        self.local_hostname = socket.gethostname() if local_hostname is None else local_hostname
        self.local_ip = socket.gethostbyname(self.local_hostname) if local_hostname is not None else extract_ip()
        logger.info(f"local hostname: {self.local_hostname} with IP: {self.local_ip}")
        self.drone_hostname = drone_hostname
        self.drone_ip = socket.gethostbyname(drone_hostname)
        logger.info(f"drone hostname: {self.drone_hostname} with IP: {self.drone_ip}")

        self.udp_int.init_sender(self.drone_ip, self.send_port)
        self.udp_int.init_receiver(self.local_ip, self.recv_port)

        self.wait_ack_duration = wait_ack_duration
        self.obs = None

        self.easy_api_cur_z = 0.0
        self.time_takeoff = time.time()

        self._lock = Lock()  # used to update the observation and also to resend
        self.__obs = None
        self.__last_sent_id = 0
        self.__last_received_id = 0
        self.__last_timestamp = time.time()
        self.__last_sent = None
        self.__wait_ack_reset = True

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

    def __listener_thread(self):
        """
        Thread in charge of receiving UDP messages.
        """
        while True:
            mes = self.udp_int.recv()
            for mb in mes:
                m = pkl.loads(mb)
                if m[0] == "ACK":  # aknowledgement type
                    print(f"DEBUG: ACK")
                    self._lock.acquire()
                    if m[1] > self.__last_received_id:
                        self.__last_received_id = m[1]  # identifier
                    self._lock.release()
                elif m[0] == "OBS":  # observation type
                    self._lock.acquire()
                    # TODO: drop outdated observations
                    self.__obs = m[1]  # observation
                    self._lock.release()
                elif m[0] == "RES":  # acknowledgement for the reset command
                    print(f"DEBUG: reset ACK")
                    self._lock.acquire()
                    self.__wait_ack_reset = False
                    self._lock.release()
                elif m[0] == "BBT":  # bad battery state
                    print(f"Low battery: {m[1]}V, drone will refuse to move")

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
            msg_type: string in ["RES", ACT"]
            msg: tuple of the form (str:command, ...:args)
        If
        """
        self._lock.acquire()
        self.__last_sent_id += 1
        self.__last_timestamp = time.time()
        data = (msg_type, self.__last_sent_id, msg) if msg is not None else (msg_type, self.__last_sent_id)
        data_s = pkl.dumps(data)
        self.__last_sent = data_s
        self._lock.release()
        self.udp_int.send(data_s)

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
                self.obs = deepcopy(self.__obs)
                self._lock.release()
                if self.obs is not None:
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
        self.send(msg_type="ACT", msg=(frame_str, x, y, z, yaw, max_velocity, max_yaw_rate, max_duration))

    # ==================================================================================================================

    # High-level API for school purpose
    # (don't use along the pro API, otherwise weird things will happen with altitude)
    # (sleeps after calls, uses cm instead of m, and uses degrees instead of rad):

    def takeoff(self, sleep_duration=10.0):
        """
        arms the drone and takes off
        """
        self.reset()
        self.arm()
        time.sleep(1.0)
        # the takeoff altitude might depend on the battery with this and Z will only be defined properly later:
        self.takeoff_nonblocking()
        self.easy_api_cur_z = EASY_API_TAKEOFF_ALTITUDE
        # this instead works (Z properly defined) but it is a bit violent and maybe not very stable:
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=0.0,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=sleep_duration,
                                      relative=False)
        time.sleep(sleep_duration)

    def land(self, sleep_duration=5.0):
        """
        lands and disarms the drone
        """
        self.easy_api_cur_z = 0.0
        self.land_nonblocking()
        time.sleep(sleep_duration)
        self.disarm()

    def forward(self, val_cm):
        """
        goes forward by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=val_m, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def backward(self, val_cm):
        """
        goes backward by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=-val_m, y=0.0, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def right(self, val_cm):
        """
        goes right by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=val_m, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def left(self, val_cm):
        """
        goes left by number of centimeters
        """
        val_m = val_cm / 100.0
        duration = val_m / EASY_API_SPEED + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=-val_m, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def up(self, val_cm):
        """
        goes up by number of centimeters
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
        time.sleep(duration)

    def down(self, val_cm):
        """
        goes down by number of centimeters
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
        time.sleep(duration)

    def cw(self, val_deg):
        """
        rotates clockwise by number of degrees (range 0-180)
        """
        val_rad = np.pi if val_deg == 180 else (val_deg % 180) * np.pi / 180.0
        duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=val_rad,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def ccw(self, val_deg):
        """
        rotates counter-clockwise by number of degrees (range 0-180)
        """
        val_rad = np.pi if val_deg == 180 else (val_deg % 180) * np.pi / 180.0
        duration = val_rad / EASY_API_YAW_RATE + EASY_API_ADDITIONAL_DURATION
        self.set_position_nonblocking(x=0.0, y=0.0, z=self.easy_api_cur_z, yaw=-val_rad,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def go(self, x, y, z, yaw=None, duration=10.0):
        self.easy_api_cur_z = clip(z, EASY_API_MIN_ALTITUDE, EASY_API_MAX_ALTITUDE)
        self.set_position_nonblocking(x=x, y=y, z=self.easy_api_cur_z, yaw=None,
                                      max_velocity=EASY_API_SPEED,
                                      max_yaw_rate=EASY_API_YAW_RATE,
                                      max_duration=duration,
                                      relative=True)
        time.sleep(duration)

    def get_time(self):
        """
        returns the number of seconds since the last takeoff command was sent
        """
        return time.time() - self.time_takeoff

    def wait(self):
        raise NotImplementedError

    def wait_for_new_battery(self):
        raise NotImplementedError

    def update(self):
        self._lock.acquire()
        self.obs = deepcopy(self.__obs)
        self._lock.release()

    def read_obs(self):
        self._lock.acquire()
        res = deepcopy(self.obs)
        self._lock.release()
        return res


if __name__ == '__main__':
    cf = Cognifly(drone_hostname="pfizer.local")
    cf.reset()
    cf.arm()
    time.sleep(2.0)
    cf.takeoff_nonblocking()
    time.sleep(10.0)

    cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=0.0,
                                max_velocity=0.5, max_yaw_rate=0.5, max_duration=5.0, relative=False)
    time.sleep(10.0)

    cf.set_position_nonblocking(x=1.0, y=0.0, z=0.5, yaw=0.0,
                                max_velocity=0.5, max_yaw_rate=0.5, max_duration=10.0, relative=True)
    time.sleep(20.0)

    cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=np.pi / 2,
                                max_velocity=0.5, max_yaw_rate=0.5, max_duration=10.0, relative=True)
    time.sleep(20.0)

    cf.set_position_nonblocking(x=1.0, y=0.0, z=0.5, yaw=0.0,
                                max_velocity=0.5, max_yaw_rate=0.5, max_duration=10.0, relative=True)
    time.sleep(20.0)

    cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=0.0,
                                max_velocity=0.5, max_yaw_rate=0.5, max_duration=10.0, relative=False)
    time.sleep(20.0)

    cf.land_nonblocking()
    time.sleep(2.0)
    cf.disarm()
    time.sleep(2.0)
