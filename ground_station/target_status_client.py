
import socket, struct, threading, time
from kernel import LIN_IP, target_status_send_port

class SendTargetStatus:
    """Comms via UDP"""

    def __init__(self, lin_ip = LIN_IP, port = target_status_send_port): 
        # udp variables
        self.addr_target = (lin_ip, port)
        self.socket_target = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # target variables
        self.target_id = None
        self.target_posi = [0, 0, 0]
        self.target_liveness = True

        # multithreading variables
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.send_target_status, daemon=True)
        self._thread.start()
        
        

    def collision_monitor_with_interceptor(self, target_id: int, collided_obj):
        if collided_obj == 'interceptor': 
            with self._lock:
                self.target_liveness = False
                self.target_id = target_id

    def current_target_posi(self, target_posi: list[3]): 
        with self._lock: self.target_posi = target_posi


    def send_target_status (self, rate = 10.0):
        """This thread sends the status of the target in a soft realtime 
        (does not require a hard realtime as other path planner models will handle more sharp and closer target status)
        packet info: unix timestamp, target id number, target position in NED, target liveness (alive = True, dead = FalseS)"""
        
        next_send_time = time.perf_counter()
        while not self._stop_event.is_set(): 

            with self._lock:
                target_status_payload = struct.pack("<di3f?", 
                                                    time.time(),
                                                    int(self.target_id or 0),
                                                    float(self.target_posi[0]),float(self.target_posi[1]),float(self.target_posi[2]),
                                                    bool(self.target_liveness))
                print("target info: ", time.time(),
                    int(self.target_id or 0),
                    float(self.target_posi[0]),float(self.target_posi[1]),float(self.target_posi[2]),
                    bool(self.target_liveness))
            self.socket_target.sendto(target_status_payload, self.addr_target)

            next_send_time += (1/rate)
            sleep_time = next_send_time - time.perf_counter()
            if sleep_time > 0: time.sleep(sleep_time) # too early (doesnt wake up determinsitically, to keep it thread light.)
            else: next_send_time = time.perf_counter() 

    def end_thread (self):
        self._stop_event.set()
        if self._thread.is_alive(): self._thread.join(timeout=1.0)
        self.socket_target.close()
                 
            
    
