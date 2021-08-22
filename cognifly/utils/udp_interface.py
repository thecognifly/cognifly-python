import socket
import logging


class UDPInterface(object):
    def __init__(self):
        self.ip_send = None
        self.port_send = None
        self.ip_recv = None
        self.port_recv = None
        self.__buffersize = None
        self.__sockO = None
        self.__sockI = None

    def init_sender(self, ip, port):
        if self.__sockO is not None:
            self.__sockO.close()
            self.__sockO = None
        self.__sockO = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip_send = ip
        self.port_send = port

    def init_receiver(self, ip, port, clean_buffer=True):
        self.ip_recv = ip
        self.port_recv = port
        if self.__sockI is None:
            self.__sockI = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            raise Exception("Cannot initialize the receiver twice.")
        self.__sockI.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__sockI.bind((ip, port))
        self.__buffersize = self.__sockI.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        logging.debug(f"buffer size: {self.__buffersize}")
        while clean_buffer:
            logging.debug(f"Cleaning receiving buffer...")
            try:
                self.__sockI.recv(1, socket.MSG_DONTWAIT)
            except IOError:  # recv raises a error when no data is received
                clean_buffer = False
        logging.debug(f"Cleaning receiving buffer...Done!")

    def __del__(self):
        if self.__sockI:
            self.__sockI.close()
        if self.__sockO:
            self.__sockO.close()

    def send(self, data):
        """
        data: binary string
        """
        assert self.__sockO is not None, "sender not initialized"

        self.__sockO.sendto(data, (self.ip_send, self.port_send))

    def recv(self, timeout=None):
        """
        This returns a list of bytestrings (all messages in the buffer)
        Returns an empty list if no message is present in the buffer
        Reads self.__buffersize bytes in the buffer
        """
        assert self.__sockI is not None, "receiver not initialized"

        if timeout:
            self.__sockI.settimeout(timeout)
        try:
            data, _ = self.__sockI.recvfrom(self.__buffersize)
            self.__sockI.settimeout(None)
        except socket.timeout:
            return []
        res = [data]
        nb = self.recv_nonblocking()
        res = res + nb
        return res

    def recv_nonblocking(self):
        """
        This returns a list of bytestrings (all messages in the buffer)
        Returns an empty list if no message is present in the buffer
        Reads everything in the buffer
        """
        assert self.__sockI is not None, "receiver not initialized"

        res = []
        while True:
            try:
                data, _ = self.__sockI.recvfrom(self.__buffersize, socket.MSG_DONTWAIT)
            except IOError:
                return res
            res += [data]


def test_send(interface, msg):
    print(f"sending: {msg}")
    interface.send(msg)


def test_recv(interface, timeout=None):
    print(f"receiving (blocking)...")
    res = interface.recv(timeout)
    print(f"received: {res}")


def test_recv_nonblocking(interface):
    print(f"receiving (non blocking)...")
    res = interface.recv_nonblocking()
    print(f"received: {res}")


def main(args):
    # standard library imports
    import time

    ip_send = args.ipsend
    ip_recv = args.iprecv
    port_send = args.portsend
    port_recv = args.portrecv

    conn = UDPInterface()
    conn.init_sender(ip_send, port_send)
    conn.init_receiver(ip_recv, port_recv)

    test_send(conn, b"hello 0")
    test_send(conn, b"hello 1")
    test_recv(conn)
    test_send(conn, b"hello 2")
    test_send(conn, b"hello 3")
    test_recv_nonblocking(conn)
    test_recv_nonblocking(conn)
    test_send(conn, b"hello 4")
    test_recv(conn, timeout=1.0)
    test_recv(conn, timeout=1.0)


if __name__ == "__main__":
    # standard library imports
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ipsend', type=str, default="127.0.0.1", help='IP address of the drone if any.')
    parser.add_argument('--iprecv', type=str, default="127.0.0.1", help='local IP address if any.')
    parser.add_argument('--portsend', type=int, default=8989, help='Port to send udp messages to.')
    parser.add_argument('--portrecv', type=int, default=8989, help='Port to reveive udp messages from.')
    args = parser.parse_args()
    main(args)
