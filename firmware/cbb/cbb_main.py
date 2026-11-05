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

import sys
import time
import ujson
import machine
import network
import espnow

CONFIG_PATH = "/cbb_config.json"
BROADCAST = b"\xff" * 6
CHANNEL = 1
VERSION = "0.1.0"


def load_config():
    try:
        with open(CONFIG_PATH) as f:
            return ujson.load(f)
    except (OSError, ValueError):
        return {}


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


def radio():
    w = network.WLAN(network.STA_IF)
    w.active(True)
    try:
        w.disconnect()
    except OSError:
        pass
    w.config(channel=CHANNEL)
    e = espnow.ESPNow()
    e.active(True)
    return w, e


def emit(obj):
    sys.stdout.write("@" + ujson.dumps(obj) + "\n")


# ---------------------------------------------------------------- bridge

def run_bridge(cfg):
    import select

    w, e = radio()
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
            emit({"id": rid, "ok": True, "role": "bridge",
                  "mac": hexmac(w.config("mac")), "version": VERSION})
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

    fput {p, o, h}: write hex chunk h at offset o of p (staged as p + ".part")
    fdone {p, n, crc}: check size and crc32 of the staged file, then move it
    into place. restore_stock puts the stock CyberBrick boot.py back.
    """

    def handle(self, cmd):
        import os
        import ubinascii

        op = cmd["op"]
        try:
            if op == "fput":
                part = cmd["p"] + ".part"
                data, off = ubinascii.unhexlify(cmd["h"]), cmd["o"]
                size = os.stat(part)[6] if off else 0
                if size == off:
                    with open(part, "ab" if off else "wb") as f:
                        f.write(data)
                elif size != off + len(data):  # equal: a retried chunk already written
                    return {"op": op, "ok": False, "err": "offset", "o": off}
                return {"op": op, "ok": True, "o": off}
            if op == "fdone":
                part = cmd["p"] + ".part"
                with open(part, "rb") as f:
                    data = f.read()
                crc = ubinascii.crc32(data) & 0xFFFFFFFF
                if len(data) != cmd["n"] or crc != cmd["crc"]:
                    return {"op": op, "ok": False, "err": "check", "n": len(data), "crc": crc}
                d = cmd["p"].rsplit("/", 1)[0]
                if d:
                    try:
                        os.mkdir(d)
                    except OSError:
                        pass
                os.rename(part, cmd["p"])
                return {"op": op, "ok": True, "p": cmd["p"]}
            if op == "restore_stock":
                os.rename("/boot_stock.py", "/boot.py")
            return {"op": op, "ok": True}
        except Exception as ex:
            return {"op": op, "ok": False, "err": str(ex)}


files = FileWriter()


def run_receiver(cfg):
    w, e = radio()
    name = cfg.get("name", "robot")
    bridge = cfg.get("bridge")
    bridge = unhexmac(bridge) if bridge else None
    timeout_ms = cfg.get("failsafe_ms", 600)
    if bridge:
        e.add_peer(bridge)

    act = Actuators()
    led = StatusLed()
    led.set(12, 6, 0)  # amber: waiting for bridge
    last_rx = time.ticks_ms()
    print("[cbb] receiver %s mac=%s bridge=%s" % (
        name, hexmac(w.config("mac")), hexmac(bridge) if bridge else None))

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
                   "version": VERSION, "moving": len(act.moving)})
        elif op == "ka":
            pass
        elif op == "batch":
            for c in cmd["cmds"]:
                apply(c)
        elif op in ("fput", "fdone", "reset", "restore_stock"):
            act.stop_all()
            reply(files.handle(cmd))
            if op in ("reset", "restore_stock") or (op == "fdone" and cmd.get("reset")):
                time.sleep_ms(200)
                machine.reset()

    while True:
        mac, msg = e.recv(20)
        if mac and mac == bridge:
            try:
                apply(ujson.loads(msg))
                last_rx = time.ticks_ms()
                led.set(0, 12, 0)
            except Exception as ex:
                print("[cbb] bad cmd:", ex)
        if act.moving and time.ticks_diff(time.ticks_ms(), last_rx) > timeout_ms:
            act.stop_all()
            led.set(12, 0, 0)  # red: failsafe tripped
            print("[cbb] failsafe stop")


def main():
    cfg = load_config()
    role = cfg.get("role")
    if role is None:
        role = "bridge" if machine.Pin(10, machine.Pin.IN).value() else "receiver"
    if role == "bridge":
        run_bridge(cfg)
    else:
        run_receiver(cfg)
