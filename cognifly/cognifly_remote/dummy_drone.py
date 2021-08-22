# Dummy script to test the remote controller without an actual drone

import pickle as pkl
from cognifly.utils.udp_interface import UDPInterface

if __name__ == '__main__':
    import time
    t_start = time.time()
    conn = UDPInterface()
    conn.init_sender('127.0.0.1', 8989)
    conn.init_receiver('127.0.0.1', 8988)
    while True:
        data_s_l = conn.recv()
        for data_s in data_s_l:
            data = pkl.loads(data_s)
            print(f"Received: {data}")
            answer = ("ACK", data[1])
            print(f"Sending: {answer}")
            conn.send(pkl.dumps(answer))
