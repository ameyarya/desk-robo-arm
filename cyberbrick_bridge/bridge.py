"""Talk to the bridge core board over USB serial."""
import itertools
import json
import queue
import threading
import time

import serial
from serial.tools import list_ports

ESPRESSIF_VID = 0x303A


class BridgeError(RuntimeError):
    pass


def find_port():
    """Return the first Espressif USB port whose board answers as a cbb bridge."""
    ports = [p.device for p in list_ports.comports() if p.vid == ESPRESSIF_VID]
    for port in sorted(ports):
        try:
            with Bridge(port):
                return port
        except (BridgeError, serial.SerialException, OSError):
            continue
    raise BridgeError("no cyberbrick-bridge board found on USB")


class Bridge:
    """Line-JSON link to the bridge board.

    Every request gets an id; the board answers with the same id. Messages a
    robot sends back arrive as {"ev": "rx", "from": mac, "msg": {...}} and are
    queued on `self.events`.
    """

    def __init__(self, port=None, timeout=2.0, connect_timeout=5.0):
        self.port = port or find_port()
        self.timeout = timeout
        s = serial.Serial()
        s.port = self.port
        s.baudrate = 115200
        s.timeout = 0.1
        s.dtr = False  # don't let opening the port reset the board
        s.rts = False
        s.open()
        self.s = s
        self._ids = itertools.count(1)
        self._pending = {}
        self._lock = threading.Lock()
        self.events = queue.Queue()
        self._closed = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._wait_ready(connect_timeout)

    def _wait_ready(self, connect_timeout):
        # The CyberBrick loader soft-resets the board when the port is first
        # opened, so the firmware may need a couple of seconds to come back.
        end = time.time() + connect_timeout
        timeout, self.timeout = self.timeout, 0.5
        try:
            while True:
                try:
                    self.ping()
                    return
                except BridgeError:
                    if time.time() > end:
                        self.close()
                        raise
        finally:
            self.timeout = timeout

    def _read_loop(self):
        buf = b""
        while not self._closed:
            try:
                buf += self.s.read(512)
            except (serial.SerialException, OSError):
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line.startswith(b"@"):
                    continue  # REPL noise / prints
                try:
                    obj = json.loads(line[1:])
                except ValueError:
                    continue
                rid = obj.get("id")
                if rid is not None and rid in self._pending:
                    self._pending.pop(rid).put(obj)
                elif "ev" in obj:
                    self.events.put(obj)

    def request(self, op, **fields):
        rid = next(self._ids)
        q = queue.Queue(maxsize=1)
        self._pending[rid] = q
        with self._lock:
            self.s.write((json.dumps({"id": rid, "op": op, **fields}) + "\n").encode())
        try:
            resp = q.get(timeout=self.timeout)
        except queue.Empty:
            self._pending.pop(rid, None)
            raise BridgeError(f"bridge did not answer {op!r} on {self.port}")
        return resp

    def ping(self):
        r = self.request("ping")
        if r.get("role") != "bridge":
            raise BridgeError(f"unexpected reply {r}")
        return r

    def send(self, to, msg):
        """Send a message to a robot (MAC hex) or "*" for all. Returns True on ack."""
        r = self.request("send", to=to, msg=msg)
        return r.get("ok", False)

    def scan(self, wait=1.0):
        """Broadcast hello and return {mac: reply} from every robot paired to this bridge."""
        self.drain()
        self.send("*", {"op": "hello"})
        found = {}
        end = time.time() + wait
        while time.time() < end:
            try:
                ev = self.events.get(timeout=max(0.0, end - time.time()))
            except queue.Empty:
                break
            if ev.get("ev") == "rx" and ev["msg"].get("op") == "hello":
                found[ev["from"]] = ev["msg"]
        return found

    def drain(self):
        while not self.events.empty():
            self.events.get_nowait()

    def close(self):
        self._closed = True
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
