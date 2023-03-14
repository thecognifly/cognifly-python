# Adapted from https://gist.github.com/rdb/8864666

import os
import struct
import array
from fcntl import ioctl
import warnings
from threading import Thread, Lock
from pathlib import Path

# These constants were borrowed from linux/input.h
axis_names = {
    0x00: 'x',
    0x01: 'y',
    0x02: 'z',
    0x03: 'rx',
    0x04: 'ry',
    0x05: 'rz',
    0x06: 'throttle',
    0x07: 'rudder',
    0x08: 'wheel',
    0x09: 'gas',
    0x0a: 'brake',
    0x10: 'hat0x',
    0x11: 'hat0y',
    0x12: 'hat1x',
    0x13: 'hat1y',
    0x14: 'hat2x',
    0x15: 'hat2y',
    0x16: 'hat3x',
    0x17: 'hat3y',
    0x18: 'pressure',
    0x19: 'distance',
    0x1a: 'tilt_x',
    0x1b: 'tilt_y',
    0x1c: 'tool_width',
    0x20: 'volume',
    0x28: 'misc',
}

button_names = {
    0x120: 'trigger',
    0x121: 'thumb',
    0x122: 'thumb2',
    0x123: 'top',
    0x124: 'top2',
    0x125: 'pinkie',
    0x126: 'base',
    0x127: 'base2',
    0x128: 'base3',
    0x129: 'base4',
    0x12a: 'base5',
    0x12b: 'base6',
    0x12f: 'dead',
    0x130: 'a',
    0x131: 'b',
    0x132: 'c',
    0x133: 'x',
    0x134: 'y',
    0x135: 'z',
    0x136: 'tl',
    0x137: 'tr',
    0x138: 'tl2',
    0x139: 'tr2',
    0x13a: 'select',
    0x13b: 'start',
    0x13c: 'mode',
    0x13d: 'thumbl',
    0x13e: 'thumbr',

    0x220: 'dpad_up',
    0x221: 'dpad_down',
    0x222: 'dpad_left',
    0x223: 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0: 'dpad_left',
    0x2c1: 'dpad_right',
    0x2c2: 'dpad_up',
    0x2c3: 'dpad_down',
}


class PS4Gamepad:
    def __init__(self, device):
        # We'll store the states here.
        self._connected = False
        self._axis_states = {}
        self._button_states = {}
        self._axis_map = []
        self._button_map = []
        self.fn = Path(device)
        self._lock = Lock()
        self._t_js = None
        self._js_loop_running = False
        self._jsdev = self._get_gamepad()
        if self._jsdev is not None:
            self._connection()
        else:
            self._disconnection()

    def _connection(self):
        try:
            with self._lock:
                # Get the device name.
                # buf = bytearray(63)
                buf = array.array('B', [0] * 64)
                ioctl(self._jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
                self.js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
                # Get number of axes and buttons.
                buf = array.array('B', [0])
                ioctl(self._jsdev, 0x80016a11, buf)  # JSIOCGAXES
                self.num_axes = buf[0]
                buf = array.array('B', [0])
                ioctl(self._jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
                self.num_buttons = buf[0]
                # Get the axis map.
                buf = array.array('B', [0] * 0x40)
                ioctl(self._jsdev, 0x80406a32, buf)  # JSIOCGAXMAP
                for axis in buf[:self.num_axes]:
                    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
                    self._axis_map.append(axis_name)
                    self._axis_states[axis_name] = 0.0
                # Get the button map.
                buf = array.array('H', [0] * 200)
                ioctl(self._jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP
                for btn in buf[:self.num_buttons]:
                    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
                    self._button_map.append(btn_name)
                    self._button_states[btn_name] = 0
                self._connected = True
                self._t_js = Thread(target=self._js_loop, daemon=True)
                self._t_js.start()
            return True

        except Exception as e:
            warnings.warn(f"caught exception {str(e)}")
            self._disconnection()
            return False

    def _disconnection(self):
        with self._lock:
            self._axis_states = {}
            self._button_states = {}
            self.js_name = None
            self.num_axes = None
            self.num_buttons = None
            self._axis_map = []
            self._button_map = []
            self._connected = False
            self._t_js = None

    def _get_gamepad(self):
        if os.path.exists(self.fn):
            try:
                js = open(self.fn, 'rb')
            except Exception as e:
                js = None
        else:
            js = None
        return js

    def _js_loop(self):
        try:
            # Main event loop
            while True:
                evbuf = self._jsdev.read(8)
                if evbuf:
                    with self._lock:
                        time, value, type, number = struct.unpack('IhBB', evbuf)

                        if type & 0x80:  # initial
                            pass

                        if type & 0x01:
                            button = self._button_map[number]
                            if button:
                                self._button_states[button] = value

                        if type & 0x02:
                            axis = self._axis_map[number]
                            if axis:
                                fvalue = value / 32767.0
                                self._axis_states[axis] = fvalue
        except OSError as e:
            pass  # just terminate the thread
        except Exception as e:
            warnings.warn(f"Caught exception {e}")
        finally:
            self._disconnection()

    def try_connect(self):
        with self._lock:
            connected = self._connected
        if not connected:
            self._jsdev = self._get_gamepad()
            if self._jsdev is not None:
                connected = self._connection()
        return connected

    def get(self):
        connected = self.try_connect()
        if not connected:
            return False, None, None
        with self._lock:
            connected = self._connected
            if connected:
                axis_states = self._axis_states.copy()
                button_states = self._button_states.copy()
            else:
                axis_states = None
                button_states = None
        return connected, axis_states, button_states


if __name__ == '__main__':
    import time
    g = PS4Gamepad(device='/dev/input/js0')
    while True:
        print(g.get())
        time.sleep(0.1)
