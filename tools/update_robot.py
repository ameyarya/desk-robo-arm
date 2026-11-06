"""Wait for a robot to answer, push the current cbb firmware to it over the radio, and check it.

  update_robot.py [--name arm | --mac MAC] [--wait 300]

Stop the web app's login service first: it holds the bridge's USB port.

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
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from make_ota_config import minify  # noqa: E402

FW = os.path.join(ROOT, 'firmware')


def version_of(path):
    for line in open(path):
        if line.startswith('VERSION = '):
            return line.split('"')[1]


def wait_for(bridge, name, until, mac=None):
    """Find the robot: by a direct (unicast, radio-retried) ping when its MAC is known, else by
    broadcast scan (no radio retries, so it can take a few tries)."""
    while time.time() < until:
        if mac:
            bridge.drain()
            if bridge.send(mac, {'op': 'ping'}):
                end = time.time() + 0.5
                while time.time() < end:
                    try:
                        ev = bridge.events.get(timeout=0.5)
                    except Exception:
                        break
                    if ev.get('ev') == 'rx' and ev.get('from') == mac and ev['msg'].get('op') == 'ping':
                        return mac, ev['msg']
            time.sleep(0.3)
            continue
        for found, info in bridge.scan(1.0).items():
            if info.get('name') == name:
                return found, info
    return None, None


PART_BYTES = 1000   # older firmware stalls its radio when one file grows past a few KB


def buffers_in_memory(version):
    """Firmware 0.2.2+ collects an upload in memory and writes it once."""
    try:
        return tuple(int(x) for x in version.split('.')) >= (0, 2, 2)
    except (AttributeError, ValueError):
        return False


def split(src):
    """Minified firmware in pieces of at most PART_BYTES, cut at line ends."""
    parts, cur = [], b''
    for line in minify(src).splitlines(keepends=True):
        if cur and len(cur) + len(line) > PART_BYTES:
            parts.append(cur)
            cur = b''
        cur += line
    if cur:
        parts.append(cur)
    return parts


def stub(n):
    """The cbb_main.py that runs the parts, in order, as one module."""
    return (
        "# cbb firmware, sent over the radio in %d small files (cbb_p0.py ...)\n"
        "def _load():\n"
        "    s = ''\n"
        "    for i in range(%d):\n"
        "        with open('/cbb/cbb_p%%d.py' %% i) as f:\n"
        "            s += f.read()\n"
        "    return s\n"
        "exec(_load(), globals())\n" % (n, n)).encode()


def push_split(bridge, mac, src):
    """For robots whose firmware writes each chunk to flash as it arrives (before 0.2.2): send
    the firmware as small files, pausing after each chunk, then a stub cbb_main.py that loads
    them. The stub goes last, so until then the robot keeps running its current firmware."""
    parts = split(src)
    print(f'older firmware on the robot: sending {len(parts)} parts of at most {PART_BYTES} bytes', flush=True)
    for i, part in enumerate(parts):
        ota.push_file(bridge, mac, part, f'/cbb/cbb_p{i}.py', progress=None, pause=0.15)
        print(f'  part {i + 1}/{len(parts)} ({len(part)} bytes) confirmed', flush=True)
    ota.push_file(bridge, mac, stub(len(parts)), '/cbb/cbb_main.py', progress=None, pause=0.15)
    bridge.send(mac, {'op': 'reset'})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--name', default='arm')
    ap.add_argument('--mac', help="the robot's radio MAC (finds it by direct ping, more reliable than a scan)")
    ap.add_argument('--wait', type=float, default=300)
    a = ap.parse_args()
    main_py = os.path.join(FW, 'cbb', 'cbb_main.py')
    target = version_of(main_py)

    with Bridge(a.port) as b:
        print(f'waiting for {a.name!r} (switch it off and on if it does not answer)...', flush=True)
        mac, info = wait_for(b, a.name, time.time() + a.wait, a.mac)
        if not mac:
            sys.exit(f'{a.name!r} did not answer')
        print('found', mac, info, flush=True)
        if info.get('version') == target:
            print(f'already on {target}')
            return
        if buffers_in_memory(info.get('version')):
            ota.push_file(b, mac, open(main_py, 'rb').read(), '/cbb/cbb_main.py')
            ota.push_file(b, mac, open(os.path.join(FW, 'boot.py'), 'rb').read(), '/boot.py', reset=True)
        else:
            push_split(b, mac, open(main_py, 'rb').read())
        print('pushed; waiting for the reboot', flush=True)
        time.sleep(4)
        mac2, info = wait_for(b, a.name, time.time() + 30, mac)
        if info and info.get('version') == target:
            print('updated:', info)
        else:
            sys.exit(f'after the update the robot answered {info!r}')


if __name__ == '__main__':
    main()
