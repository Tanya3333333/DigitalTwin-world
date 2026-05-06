
import socket, struct

class InterceptorStateServer: 
    """
    A udp server that gets the data from a different PC to keep the main drone states as deterministic/realtime as possible
    """
    def __init__(self):
        self.win_ip = "0.0.0.0"
        self.port_state = 55556
        self.socket_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_state.bind((self.win_ip, self.port_state))
        self.socket_state.setblocking(False) # make it none blocking

        self.x = 0
        self.y = 0
        self.z = 0
        self.qw = 1
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.operating_systems_dt = []

        self.latest_pose = {
            "frame_id": "DEFAULT_FRAME",
            "translation": {"x": self.x, "y": self.y, "z": self.z},
            "rotation": {"w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz}
        }

    def _states(self):
        try:
            buf_state, addr_state = self.socket_state.recvfrom(8024)
            state_data = struct.unpack("<3f4fQ", buf_state)

            self.x = state_data[0]
            self.y = state_data[1]
            self.z = state_data[2]

            self.qw = state_data[3]
            self.qx = state_data[4]
            self.qy = state_data[5]
            self.qz = state_data[6]

            self.latest_pose = {
                "frame_id": "DEFAULT_FRAME",
                "translation": {"x": self.x, "y": self.y, "z": self.z},
                "rotation": {"w": self.qw, "x": self.qx, "y": self.qy, "z": self.qz}
            }

        except BlockingIOError:
            pass

        return self.latest_pose