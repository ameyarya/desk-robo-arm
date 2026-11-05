"""Wait for a robot running the OTA loader, then push the full cbb firmware.

  finish_install.py [--port PORT] [--wait 240]

Power the robot on (or off and on) while this runs: the loader only listens
for 2 minutes after boot.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cyberbrick_bridge import Bridge, ota  # noqa: E402

FW = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "firmware")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--wait", type=float, default=240)
    a = ap.parse_args()

    with Bridge(a.port) as b:
        print("bridge", b.ping()["mac"], flush=True)
        end = time.time() + a.wait
        found = {}
        while time.time() < end and not found:
            found = b.scan(1.0)
        if not found:
            sys.exit("no robot answered; is it on and did the installer run?")
        mac, info = next(iter(found.items()))
        print("found", mac, info, flush=True)
        if info.get("version") == "loader":
            ota.push_file(b, mac, open(os.path.join(FW, "cbb", "cbb_main.py"), "rb").read(),
                          "/cbb/cbb_main.py")
            ota.push_file(b, mac, open(os.path.join(FW, "boot.py"), "rb").read(),
                          "/boot.py", reset=True)
            print("full firmware pushed; waiting for reboot", flush=True)
            time.sleep(3)
        end = time.time() + 20
        while time.time() < end:
            r = b.scan(1.0)
            if mac in r:
                print("robot running:", r[mac], flush=True)
                return
        sys.exit("robot did not come back after the update")


if __name__ == "__main__":
    main()
