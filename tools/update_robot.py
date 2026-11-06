"""Push the current cbb firmware to a robot's receiver over the radio, and optionally move both
boards to another Wi-Fi channel.

  update_robot.py --mac MAC [--name arm] [--channel 11] [--wait 600]

Stop the web app's login service first: it holds the transmitter's USB port.

- Receivers on firmware 0.2.2+ collect an upload in memory: the firmware goes as one file.
- Older receivers write each chunk to flash as it arrives: the firmware goes as ~1 KB parts plus a
  stub cbb_main.py that loads them. Confirmed parts are remembered (tools/.update-progress.json),
  so a run that fails partway resumes where it stopped. Nothing changes what the receiver runs
  until the stub is written, last.
- --channel N also writes the receiver's cbb_config.json with that channel (it switches when it
  restarts at the end), then moves the transmitter to N to check the receiver came back. Save the
  transmitter's channel afterwards with: tools/deploy.py PORT bridge --channel N
"""
import argparse
import json
import os
import sys
import time
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from cyberbrick_bridge import Bridge, ota  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from make_ota_config import minify  # noqa: E402

FW = os.path.join(ROOT, 'firmware')
PROGRESS = os.path.join(ROOT, 'tools', '.update-progress.json')
PART_BYTES = 1000   # older firmware stalls its radio when one file grows past a few KB


def version_of(path):
    for line in open(path):
        if line.startswith('VERSION = '):
            return line.split('"')[1]


def wait_for(bridge, name, until, mac=None):
    """Find the receiver: by a direct (unicast, radio-retried) ping when its MAC is known, else by
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


def receiver_config(name, transmitter_mac, channel, failsafe_ms=600):
    return json.dumps({'role': 'receiver', 'name': name, 'bridge': transmitter_mac,
                       'failsafe_ms': failsafe_ms, 'channel': channel}, separators=(',', ':')).encode()


def push_with_patience(bridge, mac, data, remote, give_up_s, pause):
    """push_file, retried whole until it works or give_up_s passes (the radio has bad spells)."""
    end = time.time() + give_up_s
    while True:
        try:
            ota.push_file(bridge, mac, data, remote, progress=None, pause=pause)
            return
        except (TimeoutError, RuntimeError) as error:
            if time.time() > end:
                raise
            print(f'    {remote}: {error}; trying again', flush=True)
            time.sleep(2)


def load_progress(key):
    try:
        with open(PROGRESS) as f:
            p = json.load(f)
        return set(p['done']) if p.get('key') == key else set()
    except (OSError, ValueError, KeyError):
        return set()


def save_progress(key, done):
    with open(PROGRESS, 'w') as f:
        json.dump({'key': key, 'done': sorted(done)}, f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port')
    ap.add_argument('--name', default='arm')
    ap.add_argument('--mac', help="the receiver's radio MAC (finds it by direct ping, more reliable than a scan)")
    ap.add_argument('--channel', type=int, help='also move the receiver (and then the transmitter) to this channel')
    ap.add_argument('--wait', type=float, default=600, help='seconds to keep trying through bad radio spells')
    ap.add_argument('--parts', action='store_true',
                    help='send the firmware as ~1 KB parts even to 0.2.2+ (when the receiver is short of memory)')
    a = ap.parse_args()
    main_py = os.path.join(FW, 'cbb', 'cbb_main.py')
    src = open(main_py, 'rb').read()
    target = version_of(main_py)

    with Bridge(a.port) as b:
        transmitter = b.ping()
        print(f"transmitter {transmitter['mac']} on channel {transmitter.get('channel', '?')}", flush=True)
        mac, info = wait_for(b, a.name, time.time() + min(a.wait, 20), a.mac)
        if not mac and a.mac:
            # firmware before 0.2.5 stops answering pings once its status no longer fits in one
            # radio packet (after ~100 s up, or after any error); a restart makes it answer again
            print('no answer to pings; restarting the receiver', flush=True)
            try:
                ota._request(b, a.mac, {'op': 'reset'}, give_up_s=20)
            except TimeoutError:
                pass
            time.sleep(6)
            mac, info = wait_for(b, a.name, time.time() + a.wait, a.mac)
        if not mac:
            sys.exit(f'receiver {a.name!r} did not answer')
        print('receiver', mac, {k: info.get(k) for k in ('version', 'up_s', 'channel')}, flush=True)
        if info.get('version') == target and (a.channel is None or info.get('channel') == a.channel):
            print(f'already on {target}' + (f', channel {a.channel}' if a.channel else ''))
            return
        deadline = a.wait
        if buffers_in_memory(info.get('version')) and not a.parts:
            push_with_patience(b, mac, src, '/cbb/cbb_main.py', deadline, 0.0)
            if a.channel:
                push_with_patience(b, mac, receiver_config(a.name, transmitter['mac'], a.channel),
                                   '/cbb_config.json', deadline, 0.0)
        else:
            parts = split(src)
            files = [(f'/cbb/cbb_p{i}.py', p) for i, p in enumerate(parts)]
            if a.channel:
                files.append(('/cbb_config.json', receiver_config(a.name, transmitter['mac'], a.channel)))
            files.append(('/cbb/cbb_main.py', stub(len(parts))))   # last: switches to the new firmware
            key = f'{mac}:{zlib.crc32(src):08x}:{a.channel}'
            done = load_progress(key)
            print(f'older firmware on the receiver: {len(files)} files of at most {PART_BYTES} bytes'
                  + (f', {len(done)} already confirmed' if done else ''), flush=True)
            for remote, data in files:
                if remote in done:
                    continue
                push_with_patience(b, mac, data, remote, deadline, 0.15)
                done.add(remote)
                save_progress(key, done)
                print(f'  {remote} ({len(data)} bytes) confirmed', flush=True)
        try:   # retried: a single lost packet must not leave the receiver on the old firmware
            ota._request(b, mac, {'op': 'reset'}, give_up_s=20)
        except TimeoutError:
            pass   # it may have restarted before its confirmation got out; the check below tells
        print('receiver restarting on the new firmware', flush=True)
        time.sleep(5)
        if a.channel:
            b.set_channel(a.channel)
            print(f'transmitter moved to channel {a.channel} (until it restarts)', flush=True)
        mac2, info = wait_for(b, a.name, time.time() + 60, mac)
        if info and info.get('version') == target:
            print('updated:', {k: info.get(k) for k in ('version', 'up_s', 'channel', 'errors', 'crash')})
            try:
                os.remove(PROGRESS)
            except OSError:
                pass
        else:
            sys.exit(f'after the update the receiver answered {info!r}')


if __name__ == '__main__':
    main()
