"""Named-joint control of one robot behind the bridge."""
import threading
import time

# How each joint is wired to the receiver shield: (kind, channel).
#   kind "motor"  -> MOTOR1/MOTOR2 port, speed -100..100
#   kind "speed"  -> PWM1..4 port, continuous (360°) servo, speed -100..100
#   kind "angle"  -> PWM1..4 port, positional (180°) servo, angle 0..180
# Verified on the L-ONE arm (2026-11-05): each joint moves the right part.
ARM_JOINTS = {
    "base": ("motor", 1),
    "shoulder": ("speed", 1),
    "elbow": ("speed", 2),
    # no gripper fitted yet; once a 180° servo is on PWM3, add:
    # "gripper": ("angle", 3),
}

KEEPALIVE_S = 0.2  # must stay well under the receiver's failsafe_ms


class Robot:
    """`keepalive=False` sends no keepalive packets: the caller must re-send motion commands
    itself while a joint should keep moving (the web server does). Keepalives refresh the
    robot's failsafe, so a lost stop plus running keepalives would keep a joint moving."""

    def __init__(self, bridge, mac, joints=None, trim=None, keepalive=True):
        self.bridge = bridge
        self.mac = mac.lower().replace(":", "")
        self.joints = dict(joints or ARM_JOINTS)
        # per-joint speed offset for continuous servos that creep at "0"
        self.trim = dict(trim or {})
        self._moving = set()
        self._stop_timers = {}
        self._lock = threading.Lock()
        if keepalive:
            self._ka = threading.Thread(target=self._keepalive, daemon=True)
            self._ka.start()

    # -- low level ---------------------------------------------------------

    def command(self, joint, value):
        """The radio message that sets `joint` to `value` (speed or angle)."""
        return self._cmd(joint, value)

    def _cmd(self, joint, value):
        kind, ch = self.joints[joint]
        if kind == "motor":
            return {"op": "motor", "ch": ch, "v": int(value)}
        if kind == "speed":
            v = int(value) + (self.trim.get(joint, 0) if value else 0)
            return {"op": "servo_speed", "ch": ch, "v": v}
        if kind == "angle":
            return {"op": "servo_angle", "ch": ch, "v": int(value)}
        raise ValueError(f"unknown joint kind {kind!r}")

    def send(self, msg):
        if not self.bridge.send(self.mac, msg):
            raise ConnectionError(f"robot {self.mac} did not ack (off or out of range?)")

    def _keepalive(self):
        while True:
            time.sleep(KEEPALIVE_S)
            if self._moving:
                try:
                    self.bridge.send(self.mac, {"op": "ka"})
                except Exception:
                    pass

    # -- joint control -----------------------------------------------------

    def move(self, joint, speed, duration=None):
        """Run a motor/continuous joint at speed -100..100.

        With `duration` (seconds) it stops by itself; without, it keeps going
        until stop() (the receiver's failsafe stops it if this process dies).
        """
        kind, _ = self.joints[joint]
        if kind == "angle":
            raise ValueError(f"{joint} is positional; use set_angle()")
        with self._lock:
            t = self._stop_timers.pop(joint, None)
            if t:
                t.cancel()
            # update before sending: a stop that gets no ack must still end the keepalives,
            # or they would keep the robot's failsafe from stopping it
            if speed:
                self._moving.add(joint)
            else:
                self._moving.discard(joint)
            self.send(self._cmd(joint, speed))
            if speed and duration:
                t = threading.Timer(duration, self.move, (joint, 0))
                t.daemon = True
                self._stop_timers[joint] = t
                t.start()

    def move_for(self, joint, speed, duration):
        """Blocking version of move(..., duration)."""
        self.move(joint, speed)
        time.sleep(duration)
        self.move(joint, 0)

    def set_angle(self, joint, angle):
        kind, _ = self.joints[joint]
        if kind != "angle":
            raise ValueError(f"{joint} is not a positional servo")
        self.send(self._cmd(joint, angle))

    def stop(self):
        with self._lock:
            for t in self._stop_timers.values():
                t.cancel()
            self._stop_timers.clear()
            self._moving.clear()
            self.send({"op": "stop"})

    def led(self, r, g, b):
        self.send({"op": "led", "rgb": [r, g, b]})

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.stop()
        except Exception:
            pass
