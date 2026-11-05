"""Wait for a robot to answer, push the current cbb firmware to it over the radio, and check it.

  update_robot.py [--name arm] [--wait 300]

Use after changing firmware/cbb/cbb_main.py. A robot that crashed into the stock firmware
won't answer: switch it off and on while this waits.
"""
import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from cyberbrick_bridge import Bridge, ota  # noqa: E402

FW = os.path.join(ROOT, 'firmware')


def version_of(path):
    for line in open(path):
        if line.startswith('VERSION = '):
            return line.split('"')[1]


def wait_for(bridge, name, until):
    while time.time() < until:
        for mac, info in bridge.scan(1.0).items():
            if info.get('name') == name:
                return mac, info
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--name', default='arm')
    ap.add_argument('--wait', type=float, default=300)
    a = ap.parse_args()
    main_py = os.path.join(FW, 'cbb', 'cbb_main.py')
    target = version_of(main_py)

    with Bridge(a.port) as b:
        print(f'waiting for {a.name!r} (switch it off and on if it does not answer)...', flush=True)
        mac, info = wait_for(b, a.name, time.time() + a.wait)
        if not mac:
            sys.exit(f'{a.name!r} did not answer')
        print('found', mac, info, flush=True)
        if info.get('version') == target:
            print(f'already on {target}')
            return
        ota.push_file(b, mac, open(main_py, 'rb').read(), '/cbb/cbb_main.py')
        ota.push_file(b, mac, open(os.path.join(FW, 'boot.py'), 'rb').read(), '/boot.py', reset=True)
        print('pushed; waiting for the reboot', flush=True)
        time.sleep(4)
        mac2, info = wait_for(b, a.name, time.time() + 30)
        if info and info.get('version') == target:
            print('updated:', info)
        else:
            sys.exit(f'after the update the robot answered {info!r}')


if __name__ == '__main__':
    main()
