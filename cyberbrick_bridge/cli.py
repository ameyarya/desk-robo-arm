"""cbb - command line for cyberbrick-bridge.

  cbb ping                         check the bridge board
  cbb scan                         list robots paired to this bridge
  cbb jog MAC motor 1 40 [-t 0.5]  raw: run MOTOR1 at 40% for 0.5 s
  cbb jog MAC speed 2 -30          raw: continuous servo on PWM2
  cbb jog MAC angle 3 90           raw: positional servo on PWM3
  cbb move MAC shoulder 30 -t 1    named joint (see robot.ARM_JOINTS)
  cbb stop MAC
"""
import argparse
import json
import os
import time

from . import ota
from .bridge import Bridge
from .robot import ARM_JOINTS, Robot


def main():
    ap = argparse.ArgumentParser(prog="cbb")
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("scan")
    j = sub.add_parser("jog")
    j.add_argument("mac")
    j.add_argument("kind", choices=["motor", "speed", "angle"])
    j.add_argument("ch", type=int)
    j.add_argument("value", type=int)
    j.add_argument("-t", "--time", type=float, default=0.5)
    m = sub.add_parser("move")
    m.add_argument("mac")
    m.add_argument("joint", choices=sorted(ARM_JOINTS))
    m.add_argument("value", type=int)
    m.add_argument("-t", "--time", type=float, default=0.5)
    s = sub.add_parser("stop")
    s.add_argument("mac")
    u = sub.add_parser("update", help="push the current firmware to a robot over the radio")
    u.add_argument("mac")
    r = sub.add_parser("restore-stock", help="put stock CyberBrick firmware back on a robot")
    r.add_argument("mac")
    a = ap.parse_args()

    with Bridge(a.port) as b:
        if a.cmd == "ping":
            print(json.dumps(b.ping()))
        elif a.cmd == "scan":
            found = b.scan()
            if not found:
                print("no robots answered")
            for mac, info in found.items():
                print(mac, json.dumps(info))
        elif a.cmd == "stop":
            Robot(b, a.mac).stop()
            print("stopped")
        elif a.cmd == "jog":
            r = Robot(b, a.mac, joints={"j": (a.kind, a.ch)})
            if a.kind == "angle":
                r.set_angle("j", a.value)
            else:
                r.move_for("j", a.value, a.time)
            print("ok")
        elif a.cmd == "update":
            fw = os.path.join(os.path.dirname(os.path.dirname(__file__)), "firmware")
            ota.push_file(b, a.mac, open(os.path.join(fw, "cbb", "cbb_main.py"), "rb").read(),
                          "/cbb/cbb_main.py")
            ota.push_file(b, a.mac, open(os.path.join(fw, "boot.py"), "rb").read(),
                          "/boot.py", reset=True)
            print("updated; robot is rebooting")
        elif a.cmd == "restore-stock":
            ota.restore_stock(b, a.mac)
            print("stock firmware restored; robot is rebooting")
        elif a.cmd == "move":
            r = Robot(b, a.mac)
            if ARM_JOINTS[a.joint][0] == "angle":
                r.set_angle(a.joint, a.value)
            else:
                r.move_for(a.joint, a.value, a.time)
            print("ok")
        time.sleep(0.05)


if __name__ == "__main__":
    main()
