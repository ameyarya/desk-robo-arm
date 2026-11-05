"""A simulated bridge + arm, for tests and `arm_server.py --simulate` (no hardware needed).

SimBridge has the parts of Bridge that ArmControl and Robot use (send, events, drain), and models
the arm on the far end of the radio:

- mode: 'cbb' (acks and answers pings like our firmware), 'stock' (acks, never replies), 'off'
  (nothing arrives, no ack).
- drop: the next N packets are lost (not delivered, no ack).
- ack_loss: the next N packets ARE delivered but their ack is lost (the sender sees "no ack").
  This is the case that once kept the real arm spinning.
- delay: every send takes this long (a slow link).
- The arm's joints and its failsafe: a moving joint stops by itself FAILSAFE_S after the last
  packet that counts. `failsafe_counts` is 'motion' (firmware 0.2.2+: only motion commands) or
  'any' (older firmware: any packet, pings included).
"""
import queue
import threading
import time

MAC = 'aabbccddeeff'
FAILSAFE_S = 0.6
MOTION_OPS = ('motor', 'servo_speed', 'servo_angle', 'batch', 'ka')


class SimBridge:
    def __init__(self, clock=time.monotonic, failsafe_counts='motion'):
        self.clock = clock
        self.failsafe_counts = failsafe_counts
        self.mode = 'cbb'
        self.drop = 0
        self.ack_loss = 0
        self.delay = 0.0
        self.sent = []
        self.events = queue.Queue()
        self.lock = threading.Lock()
        self.started = clock()
        self.arm = {}            # (kind, ch) -> speed the simulated arm is running
        self.last_counted = None
        self.failsafe_trips = 0

    # -- the radio, as Bridge --------------------------------------------------

    def send(self, mac, msg):
        if self.delay:
            time.sleep(self.delay)
        with self.lock:
            self._failsafe()
            self.sent.append(msg)
            del self.sent[:-2000]
            if self.mode == 'off' or self.drop:
                self.drop = max(0, self.drop - 1)
                return False
            self._arrive(msg)
            if self.ack_loss:
                self.ack_loss -= 1
                return False
        if self.mode == 'cbb' and msg.get('op') == 'ping':
            self.events.put({'ev': 'rx', 'from': mac,
                             'msg': {'op': 'ping', 'name': 'arm', 'version': 'sim',
                                     'up_s': int(self.clock() - self.started), 'errors': 0,
                                     'failsafe': self.failsafe_trips, 'last_error': None, 'crash': None,
                                     'mem': 60000}})
        return True

    def drain(self):
        while not self.events.empty():
            self.events.get_nowait()

    def close(self):
        pass

    # -- the arm ---------------------------------------------------------------

    def _arrive(self, msg):
        op = msg.get('op')
        if op in ('motor', 'servo_speed'):
            key = (op, msg['ch'])
            if msg['v']:
                self.arm[key] = msg['v']
            else:
                self.arm.pop(key, None)
        elif op == 'stop':
            self.arm.clear()
        if self.failsafe_counts == 'any' or op in MOTION_OPS:
            self.last_counted = self.clock()

    def _failsafe(self):
        if self.arm and self.last_counted is not None and self.clock() - self.last_counted > FAILSAFE_S:
            self.arm.clear()
            self.failsafe_trips += 1

    def arm_moving(self):
        """What the simulated arm is running right now (after its failsafe)."""
        with self.lock:
            self._failsafe()
            return dict(self.arm)

    def moves(self):
        """The motion and stop commands sent (ignoring pings)."""
        with self.lock:
            return [m for m in self.sent if m.get('op') != 'ping']


def sim_connect(bridge=None):
    """A connect() for ArmControl that always finds the simulated arm."""
    from cyberbrick_bridge import Robot
    bridge = bridge or SimBridge()

    def connect():
        return bridge, Robot(bridge, MAC, keepalive=False), {'mac': MAC, 'name': 'arm', 'version': 'sim'}

    connect.bridge = bridge
    return connect
