import socket
from psutil import process_iter
import time
import warnings


def extract_ip():
    st = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        st.connect(('10.255.255.255', 1))
        ip = st.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        st.close()
    return ip


def get_free_port(port):
    initial_port = port
    is_free = True
    while True:
        for proc in process_iter():
            try:
                for conns in proc.connections(kind='inet'):
                    if conns.laddr.port == port:
                        is_free = False
            except:
                pass
        if is_free:
            break
        else:
            is_free = True
            port += 1
            if port >= 65000:
                port = 2000
            if port == initial_port:
                return None
    return port
