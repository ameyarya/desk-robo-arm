"""A simulated bridge + arm, for tests and `arm_server.py --simulate` (no hardware needed).

SimBridge has the parts of Bridge that ArmControl and Robot use (send, events, drain). The arm on
the far end acks every packet and answers pings like the cbb firmware, unless `mode` says
otherwise: 'cbb' (normal), 'stock' (acks, never replies), 'off' (no ack). `drop` loses the next
N packets and `delay` makes every send slow.
"""
import queue
import threading
import time

MAC = 'aabbccddeeff'


class SimBridge:
    def __init__(self):
        self.mode = 'cbb'
        self.drop = 0
        self.delay = 0.0
        self.sent = []
        self.events = queue.Queue()
        self.lock = threading.Lock()
        self.started = time.monotonic()

    def send(self, mac, msg):
        if self.delay:
            time.sleep(self.delay)
        with self.lock:
            self.sent.append(msg)
            del self.sent[:-500]
            if self.mode == 'off' or self.drop:
                self.drop = max(0, self.drop - 1)
                return False
        if self.mode == 'cbb' and msg.get('op') == 'ping':
            self.events.put({'ev': 'rx', 'from': mac,
                             'msg': {'op': 'ping', 'name': 'arm', 'version': 'sim',
                                     'up_s': int(time.monotonic() - self.started), 'errors': 0,
                                     'failsafe': 0, 'last_error': None, 'crash': None, 'mem': 60000}})
        return True

    def drain(self):
        while not self.events.empty():
            self.events.get_nowait()

    def moves(self):
        """The motion commands sent (ignoring pings and keepalives)."""
        with self.lock:
            return [m for m in self.sent if m.get('op') not in ('ping', 'ka')]

    def close(self):
        pass


def sim_connect(bridge=None):
    """A connect() for ArmControl that always finds the simulated arm."""
    from cyberbrick_bridge import Robot
    bridge = bridge or SimBridge()

    def connect():
        return bridge, Robot(bridge, MAC), {'mac': MAC, 'name': 'arm', 'version': 'sim'}

    connect.bridge = bridge
    return connect
