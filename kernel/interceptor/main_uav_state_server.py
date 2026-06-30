import socket
import struct
import multiprocessing as mp
from kernel import interceptor_state_recv_port


def interceptor_udp_receiver(shared_state, stop_event, ip="0.0.0.0", port=interceptor_state_recv_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.settimeout(0.01)

    try:
        while not stop_event.is_set():
            try:
                buf, addr = sock.recvfrom(8024)
                state_data = struct.unpack("<3f4fQ", buf)

                shared_state["pose"] = {
                    "frame_id": "DEFAULT_FRAME",
                    "translation": {
                        "x": state_data[0],
                        "y": state_data[1],
                        "z": state_data[2],
                    },
                    "rotation": {
                        "w": state_data[3],
                        "x": state_data[4],
                        "y": state_data[5],
                        "z": state_data[6],
                    },
                }

                shared_state["timestamp_ns"] = state_data[7]

            except socket.timeout:
                pass

    except KeyboardInterrupt:
        pass

    finally:
        sock.close()