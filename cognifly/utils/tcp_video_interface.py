# code inspired from https://picamera.readthedocs.io/en/release-1.13/recipes2.html

import io
import socket
import struct
import time
import logging
from PIL import Image
from threading import Thread, Lock, Condition
from copy import deepcopy
import numpy as np
import traceback


class SplitFrames(object):
    def __init__(self):  # , connection
        # self.connection = connection
        self.stream = io.BytesIO()
        self.count = 0
        self._condition_out = Condition()
        self.__frame = None
        self.__framelen = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; send the old one's length
            # then the data
            size = self.stream.tell()
            if size > 0:
                with self._condition_out:
                    self.__framelen = size
                    # self.connection.write(struct.pack('<L', size))
                    # self.connection.flush()
                    self.stream.seek(0)
                    # self.connection.write(self.stream.read(size))
                    self.__frame = self.stream.read(size)
                    self._condition_out.notifyAll()
                self.count += 1
                self.stream.seek(0)
        self.stream.write(buf)

    def get_frame(self):
        return self.__framelen, self.__frame


class TCPVideoInterface(object):
    def __init__(self, camera=None):
        self.ip_send = None
        self.port_send = None
        self.__buffersize = None
        self.__sockO = None
        self.__lock = Lock()
        self.__record = False
        self.__stream_running = False
        self.__receiver_running = False
        self.__camera_error = False
        self.__camera_exception = None
        self.__camera_traceback = None
        self.__image = None
        self.__image_i = -1
        self.__receive = False
        self.condition_in = Condition()
        self.camera = camera

    def get_camera_error(self):
        with self.__lock:
            err = self.__camera_error
            exc = self.__camera_exception
            trace = self.__camera_traceback
        return err, exc, trace

    def __streaming_thread(self, ip, port, wait_duration, resolution, fps):
        camera_error = False
        connection = None
        client_socket = None
        camera = None
        try:
            import picamera  # can be imported only on the raspberry pi
            client_socket = socket.socket()
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            client_socket.connect((ip, port))
            connection = client_socket.makefile('wb')
            output = SplitFrames()  # connection
            with picamera.PiCamera(resolution=resolution, framerate=fps) as camera:
                time.sleep(2)
                with self.__lock:
                    self.__record = True
                    record = True
                camera.start_recording(output, format='mjpeg')
                while record:
                    # retrieve freshest frame
                    with output._condition_out:
                        output._condition_out.wait()
                        framelen, frame = output.get_frame()
                    # send length and frame:
                    connection.write(struct.pack('<L', framelen))
                    connection.flush()
                    connection.write(frame)
                    # camera.wait_recording(wait_duration)
                    # check whether we must keep recording:
                    with self.__lock:
                        record = self.__record
                camera.stop_recording()
                # Write the terminating 0-length to the connection to let the
                # server know we're done
                connection.write(struct.pack('<L', 0))
            if connection is not None:
                connection.close()
            if client_socket is not None:
                client_socket.close()
        except Exception as e:
            logging.info("error")
            t_e = type(e)
            if t_e != OSError and not issubclass(t_e, ConnectionError):
                with self.__lock:
                    self.__camera_error = True
                    self.__camera_exception = str(e)
                    self.__camera_traceback = ''.join(traceback.format_tb(e.__traceback__))
                # raise e
        finally:
            if camera is not None:
                camera.close()
            with self.__lock:
                self.__stream_running = False
                self.__record = False

    def start_streaming(self, ip_dest, port_dest, wait_duration=1.0, resolution='VGA', fps=10):
        """
        Starts the streaming thread on the drone.
        Caution: stop_streaming() must be called between calls to start_streaming().
        """
        with self.__lock:
            streaming_started = self.__stream_running
            if not streaming_started:
                self.__stream_running = True
        if not streaming_started:
            start_streaming_thread = Thread(target=self.__streaming_thread, args=(ip_dest, port_dest, wait_duration, resolution, fps))
            start_streaming_thread.setDaemon(True)  # thread will be terminated at exit
            start_streaming_thread.start()
        else:
            logging.info("start_streaming() called although streaming is already started.")

    def stop_streaming(self):
        """
        Ends the streaming thread on the drone
        """
        with self.__lock:
            self.__record = False

    def __stream_receiver_thread(self, port):  # display=False
        """
        TCP server, should be running prior to the client
        """
        # if display:
        #     import cv2

        server_socket = socket.socket()
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', port))
        server_socket.listen(0)

        # Accept a single connection and make a file-like object out of it
        connection = server_socket.accept()[0].makefile('rb')
        try:
            while True:
                # Read the length of the image as a 32-bit unsigned int. If the
                # length is zero, quit the loop
                image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break
                # Construct a stream to hold the image data and read the image
                # data from the connection
                image_stream = io.BytesIO()
                image_stream.write(connection.read(image_len))
                # Rewind the stream, open it as an image with PIL and do some
                # processing on it
                image_stream.seek(0)
                image = Image.open(image_stream)

                # if display:
                #     open_cv_image = np.array(image)
                #     im = open_cv_image[:, :, ::-1].copy()
                #     cv2.imshow("Stream", im)
                #     cv2.waitKey(1)

                with self.condition_in:
                    with self.__lock:
                        self.__image_i += 1
                        self.__image = image
                        receive = self.__receive
                    self.condition_in.notifyAll()
                if not receive:
                    break
        finally:
            # if display:
            #     cv2.destroyAllWindows()
            connection.close()
            server_socket.close()
            with self.__lock:
                self.__receiver_running = False

    def start_receiver(self, port, display):
        with self.__lock:
            self.__receive = True
            receiver_running = self.__receiver_running
            self.__receiver_running = True
        if not receiver_running:
            receiver_thread = Thread(target=self.__stream_receiver_thread, args=(port, ))
            receiver_thread.setDaemon(True)  # thread will be terminated at exit
            receiver_thread.start()

    def stop_receiver(self):
        with self.__lock:
            self.__receive = False
            receiver_running = self.__receiver_running
        while receiver_running:
            with self.__lock:
                receiver_running = self.__receiver_running
            if receiver_running:
                time.sleep(0.01)

    def get_last_image(self, min_image_i=-1):
        """
        Outputs the last image and its identifier when it is more recent than the image with the inf_image_i identifier
        Args:
            min_image_i: int: identifier of the previously received image (starts at 0 and increases)
        Returns:
            im: PIL.Image or None: None when the current identifier is not > inf_image_i
            im_i: int: identifier
        """
        with self.__lock:
            im_i = self.__image_i
            data = deepcopy(self.__image) if im_i > min_image_i else None
        if data is not None:
            im = data
            pil_image = data.convert('RGB')
            open_cv_image = np.array(pil_image)
            im = open_cv_image[:, :, ::-1].copy()
        else:
            im = None
        return im, im_i

    def __del__(self):
        if self.__sockO:
            try:
                self.__sockO.close()
            except BrokenPipeError:
                pass
        self.stop_receiver()
