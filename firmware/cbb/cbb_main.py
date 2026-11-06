# CyberBrick bridge firmware: one file, two roles.
#
#   bridge   - core board on the Mac's USB. Reads JSON lines from USB serial,
#              forwards them over ESP-NOW, and prints replies back as
#              "@{json}" lines.
#   receiver - core board on a robot's shield. Takes commands only from its
#              configured bridge and drives the shield's servos and motors.
#
# Role comes from /cbb_config.json, falling back to the shield's role pin
# (pin 10 reads high on a bare board, low on a receiver shield).

import gc
import sys
import time
import ujson
import machine
import network
import espnow

CONFIG_PATH = "/cbb_config.json"
CRASH_PATH = "/cbb_crash.txt"
MOTION_OPS = ("motor", "servo_speed", "servo_angle", "batch", "ka")
BROADCAST = b"\xff" * 6
CHANNEL = 1   # default Wi-Fi channel; cbb_config.json "channel" overrides it (both boards must match)
VERSION = "0.2.3"


def load_config():
    try:
        with open(CONFIG_PATH) as f:
            return ujson.load(f)
    except (OSError, ValueError):
        return {}


def fs_free():
    """Free bytes on the board's flash filesystem."""
    try:
        import os
        st = os.statvfs("/")
        return st[0] * st[3]
    except Exception:
        return None


def hexmac(b):
    return "".join("%02x" % x for x in b)


def unhexmac(s):
    return bytes(int(s[i:i + 2], 16) for i in range(0, 12, 2))


class StatusLed:
    def __init__(self):
        try:
            import neopixel
            self.np = neopixel.NeoPixel(machine.Pin(8, machine.Pin.OUT), 1)
        except Exception:
            self.np = None
        self.rgb = None

    def set(self, r, g, b):
        if self.np is None or self.rgb == (r, g, b):
            return
        self.rgb = (r, g, b)
        self.np[0] = self.rgb
        self.np.write()


def radio(channel=CHANNEL):
    w = network.WLAN(network.STA_IF)
    w.active(True)
    try:
        w.disconnect()
    except OSError:
        pass
    w.config(channel=channel)
    try:
        w.config(pm=w.PM_NONE)  # no Wi-Fi power saving: it drops ESP-NOW packets
    except (AttributeError, ValueError, OSError):
        pass
    e = espnow.ESPNow()
    e.active(True)
    return w, e


def emit(obj):
    sys.stdout.write("@" + ujson.dumps(obj) + "\n")


# ---------------------------------------------------------------- bridge

def run_bridge(cfg):
    import select

    w, e = radio(cfg.get("channel", CHANNEL))
    e.add_peer(BROADCAST)
    peers = set()
    led = StatusLed()
    led.set(0, 0, 12)

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    line = ""
    emit({"ev": "ready", "role": "bridge", "mac": hexmac(w.config("mac")),
          "version": VERSION})

    def handle(req):
        rid = req.get("id")
        op = req.get("op")
        if op == "ping":
            emit({"id": rid, "ok": True, "role": "bridge", "channel": w.config("channel"),
                  "mac": hexmac(w.config("mac")), "version": VERSION})
        elif op == "channel":  # switch radio channel now (the receiver must be on the same one)
            w.config(channel=int(req["ch"]))
            emit({"id": rid, "ok": True, "channel": w.config("channel")})
        elif op == "send":
            to = req["to"]
            mac = BROADCAST if to == "*" else unhexmac(to)
            if mac != BROADCAST and mac not in peers:
                e.add_peer(mac)
                peers.add(mac)
            msg = ujson.dumps(req["msg"])
            if len(msg) > espnow.MAX_DATA_LEN:
                emit({"id": rid, "ok": False, "err": "msg too long"})
                return
            # sync send waits for the receiver's MAC-level ack (unicast only)
            try:
                ok = e.send(mac, msg, mac != BROADCAST)
            except OSError as ex:
                emit({"id": rid, "ok": False, "err": "send: %s" % ex})
                return
            emit({"id": rid, "ok": bool(ok) or mac == BROADCAST,
                  "err": None if ok or mac == BROADCAST else "no ack"})
        else:
            emit({"id": rid, "ok": False, "err": "unknown op %s" % op})

    while True:
        try:
            while poll.poll(0):
                c = sys.stdin.read(1)
                if c == "\n":
                    if line.strip():
                        try:
                            handle(ujson.loads(line))
                        except Exception as ex:
                            emit({"ok": False, "err": "bad request: %s" % ex})
                    line = ""
                elif c != "\r":
                    line += c
            mac, msg = e.recv(5)
            if mac:
                try:
                    emit({"ev": "rx", "from": hexmac(mac), "msg": ujson.loads(msg)})
                except ValueError:
                    pass
        except Exception as ex:  # stay up: the Mac can't reach a crashed bridge
            emit({"ev": "error", "err": "%r" % ex})
            time.sleep_ms(50)


# -------------------------------------------------------------- receiver

class Actuators:
    def __init__(self):
        sys.path.append("/bbl")
        from servos import ServosController
        from motors import MotorsController
        self.servos = ServosController()
        self.motors = MotorsController()
        self.moving = set()
        self.stop_all()

    def servo_angle(self, ch, v):
        self.moving.discard(("s", ch))
        self.servos.set_angle(ch, max(0, min(180, int(v))))

    def servo_speed(self, ch, v):
        v = max(-100, min(100, int(v)))
        if v == 0:
            self.servos.stop(ch)  # no pulse: continuous servo idles
            self.moving.discard(("s", ch))
        else:
            self.servos.set_speed(ch, v)
            self.moving.add(("s", ch))

    def motor(self, ch, v):
        v = max(-100, min(100, int(v)))
        if v == 0:
            self.motors.stop(ch)  # brake
            self.moving.discard(("m", ch))
        else:
            self.motors.set_speed(ch, v * 2048 // 100)
            self.moving.add(("m", ch))

    def stop_all(self):
        for ch in (1, 2, 3, 4):
            self.servos.stop(ch)
        for ch in (1, 2):
            self.motors.stop(ch)
        self.moving.clear()


class FileWriter:
    """Over-the-air file updates, so a robot never needs USB after install.

    fput {p, o, h}: add hex chunk h at offset o of p. Chunks are collected in memory, because
    appending to a flash file rewrites a 4 KB block each time and the radio misses packets
    while that happens; the file is written once, by fdone.
    fdone {p, n, crc}: check size and crc32, write the file (staged as p + ".part") and move it
    into place. fdel {p} removes a file. restore_stock puts the stock CyberBrick boot.py back.
    """

    def __init__(self):
        self.path = None
        self.buf = bytearray()

    def handle(self, cmd):
        import os
        import ubinascii

        op = cmd["op"]
        try:
            if op == "fput":
                data, off = ubinascii.unhexlify(cmd["h"]), cmd["o"]
                if off == 0:
                    self.path, self.buf = cmd["p"], bytearray()
                if self.path != cmd["p"]:
                    return {"op": op, "ok": False, "err": "start at offset 0", "o": off}
                if len(self.buf) == off:
                    self.buf.extend(data)
                elif len(self.buf) != off + len(data):  # equal: a retried chunk already added
                    return {"op": op, "ok": False, "err": "offset", "o": off, "have": len(self.buf)}
                return {"op": op, "ok": True, "o": off}
            if op == "fdone":
                if self.path != cmd["p"]:
                    return {"op": op, "ok": False, "err": "no upload of %s in progress" % cmd["p"]}
                crc = ubinascii.crc32(self.buf) & 0xFFFFFFFF
                if len(self.buf) != cmd["n"] or crc != cmd["crc"]:
                    return {"op": op, "ok": False, "err": "check", "n": len(self.buf), "crc": crc}
                d = cmd["p"].rsplit("/", 1)[0]
                if d:
                    try:
                        os.mkdir(d)
                    except OSError:
                        pass
                part = cmd["p"] + ".part"
                with open(part, "wb") as f:
                    f.write(self.buf)
                os.rename(part, cmd["p"])
                self.path, self.buf = None, bytearray()
                return {"op": op, "ok": True, "p": cmd["p"]}
            if op == "fdel":  # remove a file and any staged copy of it
                for path in (cmd["p"], cmd["p"] + ".part"):
                    try:
                        os.remove(path)
                    except OSError:
                        pass
                return {"op": op, "ok": True, "free": fs_free()}
            if op == "restore_stock":
                os.rename("/boot_stock.py", "/boot.py")
            return {"op": op, "ok": True}
        except Exception as ex:
            return {"op": op, "ok": False, "err": str(ex)}


files = FileWriter()


def run_receiver(cfg):
    w, e = radio(cfg.get("channel", CHANNEL))
    name = cfg.get("name", "robot")
    bridge = cfg.get("bridge")
    bridge = unhexmac(bridge) if bridge else None
    timeout_ms = cfg.get("failsafe_ms", 600)
    if bridge:
        e.add_peer(bridge)

    act = Actuators()
    led = StatusLed()
    led.set(12, 6, 0)  # amber: waiting for bridge
    boot_ms = last_rx = time.ticks_ms()
    # reported in hello so the Mac can see trouble the board can't print without USB;
    # crash is the traceback boot.py saved when a previous run fell over
    health = {"errors": 0, "failsafe": 0, "last_error": None, "crash": None}
    try:
        with open(CRASH_PATH) as f:
            health["crash"] = f.read()[-160:]
    except OSError:
        pass

    def reply(obj):
        try:
            e.send(bridge, ujson.dumps(obj), False)
        except OSError:
            pass

    def apply(cmd):
        op = cmd.get("op")
        if op == "servo_angle":
            act.servo_angle(cmd["ch"], cmd["v"])
        elif op == "servo_speed":
            act.servo_speed(cmd["ch"], cmd["v"])
        elif op == "motor":
            act.motor(cmd["ch"], cmd["v"])
        elif op == "stop":
            act.stop_all()
        elif op == "led":
            led.set(*cmd["rgb"])
        elif op in ("hello", "ping"):
            reply({"op": op, "seq": cmd.get("seq"), "name": name,
                   "version": VERSION, "moving": len(act.moving), "channel": w.config("channel"),
                   "up_s": time.ticks_diff(time.ticks_ms(), boot_ms) // 1000,
                   "errors": health["errors"], "failsafe": health["failsafe"],
                   "last_error": health["last_error"], "crash": health["crash"],
                   "mem": gc.mem_free(), "fs_free": fs_free()})
            gc.collect()
        elif op == "ka":
            pass
        elif op == "batch":
            for c in cmd["cmds"]:
                apply(c)
        elif op in ("fput", "fdone", "fdel", "reset", "restore_stock"):
            act.stop_all()
            reply(files.handle(cmd))
            if op in ("reset", "restore_stock") or (op == "fdone" and cmd.get("reset")):
                time.sleep_ms(200)
                machine.reset()

    # Nothing may leave this loop: an exception out of main() drops the board to
    # the stock firmware (boot.py's fallback), which the bridge can't talk to.
    while True:
        try:
            mac, msg = e.recv(20)
            if mac and mac == bridge:
                try:
                    cmd = ujson.loads(msg)
                    apply(cmd)
                    # only motion commands (and a library keepalive) hold off the failsafe:
                    # pings and status checks must never keep a joint moving
                    if cmd.get("op") in MOTION_OPS:
                        last_rx = time.ticks_ms()
                    led.set(0, 12, 0)
                except Exception as ex:
                    health["errors"] += 1
                    health["last_error"] = "cmd: %r" % ex
            if act.moving and time.ticks_diff(time.ticks_ms(), last_rx) > timeout_ms:
                act.stop_all()
                health["failsafe"] += 1
                led.set(12, 0, 0)  # red: failsafe tripped
        except Exception as ex:
            health["errors"] += 1
            health["last_error"] = "loop: %r" % ex
            try:
                act.stop_all()
            except Exception:
                pass
            time.sleep_ms(50)


def main():
    cfg = load_config()
    role = cfg.get("role")
    if role is None:
        role = "bridge" if machine.Pin(10, machine.Pin.IN).value() else "receiver"
    if role == "bridge":
        run_bridge(cfg)
    else:
        run_receiver(cfg)
