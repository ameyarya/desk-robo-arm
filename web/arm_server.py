"""Robo Arm web app: drive the L-ONE arm's joints from a browser.

Run: .venv/bin/python web/arm_server.py
Then open http://robo-arm.local:8020/ from any device on the home Wi-Fi (advertised over mDNS),
or http://localhost:8020/ on this Mac. Other devices log in with the password in
web/.dashboard-password (created on first run, gitignored); this Mac needs no login. STOP works
without logging in. The page is web/arm.html.

- Owns the bridge board's USB port (the drive server of the rover project can't run at the same
  time) and finds the arm over the radio by name.
- Page requests only change what the arm should do; one worker thread does all the radio work,
  so the page never waits on the radio.
- Arm: the page holds a 3 s lease and renews it every second; when it lapses, or the page
  closes, everything stops and it disarms.
- Hold-to-move: while armed, the page renews each joint's hold (with its speed) every 150 ms;
  0.35 s after the last renewal that joint stops. The arm's own failsafe stops it 0.6 s after
  the radio goes quiet.
- Link: the worker pings the arm every second. One lost packet changes nothing; the arm counts
  as lost (and everything disarms) only after LOST_AFTER failed pings in a row.
"""
import argparse
import hashlib
import hmac
import json
import queue
import secrets
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from cyberbrick_bridge import ARM_JOINTS, Bridge, BridgeError, Robot  # noqa: E402

PAGE_FILE = ROOT / 'arm.html'
PASSWORD_FILE = ROOT / '.dashboard-password'
COOKIE = 'robo_arm_session'
LEASE_S = 3.0
HOLD_S = 0.35
MAX_SPEED = 80          # the page's speed settings go up to this; the server clamps to it too
SPEED_STEP = 5          # joystick speeds are rounded to this, so tiny wobbles don't flood the radio
PING_S = 1.0
PING_REPLY_S = 0.4
LOST_AFTER = 3          # failed pings in a row before the arm counts as lost
RECONNECT_S = 2.0
RETRY_S = 0.1           # after a send the arm didn't ack, wait this long before trying again
LOGIN_PAGE = '''<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Robo Arm</title><style>:root{color-scheme:dark}body{margin:0;min-height:100vh;display:grid;place-items:center;
background:#10151d;color:#edf2fa;font-family:system-ui;padding:16px}form{background:#19212d;border:1px solid #2c3748;
border-radius:12px;padding:20px;display:grid;gap:10px;width:min(320px,100%)}h1{font-size:20px;margin:0}
input,button{font:inherit;color:inherit;background:#101722;border:1px solid #40516b;border-radius:8px;padding:10px}
button{background:#277b58;border-color:#46a477;cursor:pointer}p{margin:0;font-size:12px;color:#ff8b80}</style>
<form method=post action=/login><h1>Robo Arm</h1><input type=password name=password placeholder=Password autofocus
autocomplete=current-password>{message}<button>Log in</button></form>'''


class Refused(Exception):
    """A request the arm's state doesn't allow (e.g. moving while disarmed)."""


def ping(bridge, mac, timeout=PING_REPLY_S):
    """Ask the robot for its hello. Returns its reply, or None (with why) if it didn't answer.

    A board that acks the packet but never replies is running other firmware (the stock
    CyberBrick firmware after a crash fallback).
    """
    bridge.drain()
    if not bridge.send(mac, {'op': 'ping'}):
        return None, 'no radio ack'
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            ev = bridge.events.get(timeout=max(0.0, end - time.monotonic()))
        except queue.Empty:
            break
        if ev.get('ev') == 'rx' and ev.get('from') == mac and ev['msg'].get('op') == 'ping':
            return ev['msg'], None
    return None, 'acked but no reply'


class ArmControl:
    """Arm lease, per-joint holds and the radio link, independent of HTTP.

    Request methods (set_armed, hold, release, stop, status) only touch state under `lock` and
    return at once. `service()` (the worker thread) does every radio call: connecting, pinging
    and sending the joint speeds the state asks for. Tests drive tick()/service() by hand with
    a fake clock and fake radio.

    `connect()` returns (bridge, robot, info) or raises; it is retried every RECONNECT_S.
    `pinger(bridge, mac)` returns (reply, why) as ping() does.
    """

    def __init__(self, connect, joints=ARM_JOINTS, clock=time.monotonic, pinger=ping):
        self.connect = connect
        self.pinger = pinger
        self.joints = [j for j, (kind, _) in joints.items() if kind != 'angle']
        self.clock = clock
        self.lock = threading.Lock()
        self.bridge = self.robot = None
        self.armed = False
        self.lease_until = 0.0
        self.holds = {}          # joint -> (speed, until): what the page asks for
        self.running = {}        # joint -> speed the arm last acknowledged
        self.stop_pending = False
        self.link = {'state': 'connecting', 'bridge': False, 'mac': None, 'name': None, 'version': None,
                     'misses': 0, 'why': None, 'health': None, 'rtt_ms': None}
        self.error = None
        self.last = 'nothing yet'
        self.next_ping = 0.0
        self.next_connect = 0.0
        self.retry_at = 0.0

    # -- page requests (fast, never touch the radio) --------------------------

    def set_armed(self, armed, renew=False):
        with self.lock:
            if armed:
                if self.link['state'] != 'ok':
                    raise Refused(self._link_problem() or 'the arm is not connected')
                if renew and not self.armed:
                    raise Refused('disarmed: ' + self.last)
                self.armed = True
                self.lease_until = self.clock() + LEASE_S
                if not renew:
                    self.last = 'armed'
            else:
                self._stop_locked('disarmed')
            return self._status_locked()

    def hold(self, joint, speed):
        with self.lock:
            if joint not in self.joints:
                raise Refused(f'unknown joint {joint!r}')
            if not self.armed:
                raise Refused('arm first')
            speed = int(round(float(speed) / SPEED_STEP) * SPEED_STEP)
            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            if speed:
                self.holds[joint] = (speed, self.clock() + HOLD_S)
            else:
                self.holds.pop(joint, None)
            return self._status_locked()

    def release(self, joint):
        with self.lock:
            self.holds.pop(joint, None)
            return self._status_locked()

    def stop(self, reason='STOP'):
        with self.lock:
            self._stop_locked(reason)
            return self._status_locked()

    def status(self):
        with self.lock:
            return self._status_locked()

    # -- worker ----------------------------------------------------------------

    def tick(self):
        """Expire the lease and holds (state only)."""
        with self.lock:
            now = self.clock()
            if self.armed and now > self.lease_until:
                self._stop_locked('lease lapsed (page closed or lost its connection)')
            for joint, (_, until) in list(self.holds.items()):
                if now > until:
                    del self.holds[joint]

    def service(self):
        """All radio work: connect, ping, and send whatever the state asks for."""
        now = self.clock()
        if self.robot is None:
            if now >= self.next_connect:
                self.next_connect = now + RECONNECT_S
                self._connect()
            return
        if self.link['state'] != 'ok':
            with self.lock:
                self.stop_pending = False   # nothing to send to; the arm's failsafe has it stopped
        elif now >= self.retry_at:
            with self.lock:
                stop = self.stop_pending
                self.stop_pending = False
                want = {j: s for j, (s, _) in self.holds.items()}
                running = dict(self.running)
            if stop:
                if self._radio(lambda: self.robot.stop(), on_ok=lambda: self.running.clear()):
                    running = {}
                else:
                    with self.lock:
                        self.stop_pending = True   # STOP is retried until the arm acks it
            for joint in self.joints:
                speed = want.get(joint, 0)
                if running.get(joint, 0) != speed:
                    # a failed send is retried shortly (running stays different until acked)
                    self._radio(lambda: self.robot.move(joint, speed),
                                on_ok=lambda: self.running.__setitem__(joint, speed))
        if now >= self.next_ping:
            self.next_ping = now + PING_S
            self._ping()

    def run(self):
        while True:
            try:
                self.tick()
                self.service()
            except Exception as error:   # keep the watchdog alive whatever happens
                with self.lock:
                    self.error = f'internal: {error}'
            time.sleep(0.03)

    # -- internals -----------------------------------------------------------

    def _radio(self, call, on_ok):
        try:
            call()
        except Exception as error:
            with self.lock:
                self.error = str(error)
            self.retry_at = self.clock() + RETRY_S
            return False
        with self.lock:
            on_ok()
            if self.error and self.link['state'] == 'ok':
                self.error = None
        return True

    def _connect(self):
        try:
            bridge, robot, info = self.connect()
        except Exception as error:
            with self.lock:
                # a BridgeError from the scan means the bridge is fine but no arm answered
                found_bridge = isinstance(error, BridgeError) and ('robot' in str(error) or 'loader' in str(error))
                self.link.update(state='connecting', why=str(error), bridge=found_bridge)
                self.error = str(error)
            return
        with self.lock:
            self.bridge, self.robot = bridge, robot
            self.link.update(state='ok', bridge=True, misses=0, why=None, **info)
            self.error = None

    def _ping(self):
        t = time.monotonic()
        try:
            reply, why = self.pinger(self.bridge, self.robot.mac)
        except Exception as error:   # the bridge itself failed (unplugged?)
            reply, why = None, f'bridge: {error}'
        with self.lock:
            if reply is not None:
                self.link.update(state='ok', misses=0, why=None, version=reply.get('version'),
                                 rtt_ms=round((time.monotonic() - t) * 1000),
                                 health={k: reply.get(k) for k in
                                         ('up_s', 'errors', 'failsafe', 'last_error', 'crash', 'mem')})
                if self.error and self.error.startswith('arm lost'):
                    self.error = None
                return
            self.link['misses'] += 1
            self.link['why'] = why
            if self.link['misses'] >= LOST_AFTER and self.link['state'] == 'ok':
                # acks without replies: our firmware isn't running. If it replied earlier
                # this session it hung or crashed; otherwise it may be the stock firmware.
                self.link['state'] = 'silent' if why == 'acked but no reply' else 'lost'
                if self.armed or self.holds:
                    self._stop_locked('arm lost')
                self.running = {}   # the arm's own failsafe has stopped it by now
                self.error = 'arm lost: ' + self._link_problem()
            if why and why.startswith('bridge:'):
                self.link['bridge'] = False
                if self.link['misses'] >= LOST_AFTER:
                    self._drop_bridge_locked()

    def _drop_bridge_locked(self):
        """Forget a failed bridge (unplugged, or its port died) so the worker reopens it."""
        self.robot = self.bridge = None
        self.running = {}
        self.link.update(state='connecting', bridge=False, why='bridge board lost; reconnecting')
        reset = getattr(self.connect, 'reset', None)
        if reset:
            try:
                reset()
            except Exception:
                pass

    def _link_problem(self):
        state = self.link['state']
        if state == 'silent':
            return ('the arm acknowledges the radio but its firmware has stopped replying (hung, or '
                    'crashed into the stock CyberBrick firmware). Switch the arm off and on.')
        if state == 'lost':
            return f"the arm stopped answering ({self.link['why']}): is it switched on?"
        if state == 'connecting':
            return self.link['why'] or 'looking for the arm'
        return None

    def _stop_locked(self, reason):
        self.armed = False
        self.holds.clear()
        self.stop_pending = True
        self.last = reason

    def _status_locked(self):
        return {'armed': self.armed, 'joints': self.joints, 'running': dict(self.running),
                'wanted': {j: s for j, (s, _) in self.holds.items()}, 'link': dict(self.link),
                'problem': self._link_problem(), 'error': self.error, 'last': self.last,
                'max_speed': MAX_SPEED}


def make_connect(port, name, mac):
    """Open the bridge once, then find the arm by MAC or by name over the radio."""
    state = {'bridge': None}

    def connect():
        if state['bridge'] is None:
            try:
                state['bridge'] = Bridge(port)
            except Exception as error:
                raise OSError(f'bridge board not found on USB ({error})') from None
        bridge = state['bridge']
        found = bridge.scan(1.0)
        for robot_mac, info in found.items():
            if (mac and robot_mac == mac) or (not mac and info.get('name') == name):
                if info.get('version') == 'loader':
                    raise BridgeError(f'{robot_mac} is still running the install loader; run tools/finish_install.py')
                robot = Robot(bridge, robot_mac)
                return bridge, robot, {'mac': robot_mac, 'name': info.get('name'), 'version': info.get('version')}
        raise BridgeError(f'no robot named {name!r} answered (is it on? if it was just restarted, give it 5 s)'
                          + (f'; saw {sorted(found)}' if found else ''))

    def reset():
        bridge, state['bridge'] = state['bridge'], None
        if bridge is not None:
            bridge.close()

    connect.reset = reset
    return connect


def dashboard_password():
    """The LAN login password: web/.dashboard-password, created with a random one if missing."""
    if not PASSWORD_FILE.exists():
        PASSWORD_FILE.write_text(secrets.token_urlsafe(9) + '\n')
        PASSWORD_FILE.chmod(0o600)
    return PASSWORD_FILE.read_text().strip()


def session_token(password):
    # Stateless: stays valid across restarts until the password changes.
    return hmac.new(password.encode(), b'robo-arm-session', hashlib.sha256).hexdigest()


class QuietServer(ThreadingHTTPServer):
    """Browsers drop poll connections mid-request all the time; don't print tracebacks for that."""
    daemon_threads = True

    def handle_error(self, request, client_address):
        if not isinstance(sys.exc_info()[1], (ConnectionResetError, BrokenPipeError)):
            super().handle_error(request, client_address)


def make_handler(control, password, local=('127.0.0.1', '::1')):
    token = session_token(password)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def authorized(self):
            if self.client_address[0] in local:
                return True
            for part in self.headers.get('Cookie', '').split(';'):
                name, _, value = part.strip().partition('=')
                if name == COOKIE and hmac.compare_digest(value, token):
                    return True
            return False

        def login(self):
            size = int(self.headers.get('Content-Length', 0))
            fields = urllib.parse.parse_qs(self.rfile.read(size).decode(errors='replace'))
            if hmac.compare_digest(fields.get('password', [''])[0].encode(), password.encode()):
                self.send_response(303)
                self.send_header('Location', '/')
                self.send_header('Set-Cookie', f'{COOKIE}={token}; Path=/; Max-Age=2592000; HttpOnly; SameSite=Strict')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            time.sleep(1)  # slow down guessing
            self.reply(401, LOGIN_PAGE.replace('{message}', '<p>Wrong password</p>').encode(), 'text/html; charset=utf-8')

        def reply(self, status, body, content_type='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def body(self):
            size = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(size) if size else b''
            try:
                return json.loads(raw or b'{}')
            except ValueError:
                return {}

        def do_GET(self):
            path = self.path.split('?')[0]
            if not self.authorized():
                return self.reply(200, LOGIN_PAGE.replace('{message}', '').encode(), 'text/html; charset=utf-8')
            if path == '/':
                return self.reply(200, PAGE_FILE.read_bytes(), 'text/html; charset=utf-8')
            if path == '/status':
                return self.reply(200, control.status())
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if self.path == '/login':
                return self.login()
            if self.path == '/stop':   # anyone may STOP
                return self.reply(200, control.stop(str(self.body().get('from', 'STOP'))[:60]))
            if not self.authorized():
                return self.reply(401, {'error': 'log in first'})
            body = self.body()
            try:
                if self.path == '/arm':
                    return self.reply(200, control.set_armed(bool(body.get('armed')), bool(body.get('renew'))))
                if self.path == '/hold':
                    return self.reply(200, control.hold(body.get('joint'), body.get('speed', 0)))
                if self.path == '/release':
                    return self.reply(200, control.release(body.get('joint')))
            except Refused as error:
                return self.reply(409, {'error': str(error), 'status': control.status()})
            except (ValueError, TypeError) as error:
                return self.reply(400, {'error': str(error)})
            self.reply(404, {'error': 'not found'})

    return Handler


def lan_ip():
    """This Mac's address on the home network (no packets are sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(('192.168.1.1', 80))
        return s.getsockname()[0]


def advertise(name, port, ip):
    """Publish <name>.local -> ip (and an _http._tcp service) over mDNS for this process's life."""
    return subprocess.Popen(['dns-sd', '-P', 'Robo Arm', '_http._tcp', 'local', str(port), f'{name}.local', ip],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8020)
    parser.add_argument('--bind', default='0.0.0.0', help='listen address (127.0.0.1 = this Mac only)')
    parser.add_argument('--name', default='robo-arm', help='mDNS name: http://<name>.local:<port>/')
    parser.add_argument('--usb', help='bridge board serial port (default: find it)')
    parser.add_argument('--robot', default='arm', help="the arm's name on the radio (cbb_config.json)")
    parser.add_argument('--mac', help="the arm's radio MAC, instead of finding it by name")
    parser.add_argument('--simulate', action='store_true', help='no hardware: a simulated arm (for trying the page)')
    args = parser.parse_args()

    if args.simulate:
        from sim import sim_connect
        connect = sim_connect()
    else:
        connect = make_connect(args.usb, args.robot, args.mac)
    control = ArmControl(connect)
    threading.Thread(target=control.run, daemon=True).start()
    server = QuietServer((args.bind, args.port), make_handler(control, dashboard_password()))
    mdns = None
    if args.bind not in ('127.0.0.1', 'localhost'):
        ip = lan_ip()
        mdns = advertise(args.name, args.port, ip)
        print(f'Robo Arm on http://{args.name}.local:{args.port}/ ({ip}) and http://localhost:{args.port}/'
              f'  (LAN login password in web/{PASSWORD_FILE.name})', flush=True)
    else:
        print(f'Robo Arm on http://localhost:{args.port}/' + ('  (SIMULATED arm)' if args.simulate else ''), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        control.stop('server stopped')
        control.service()   # send the stop before exiting
        if mdns:
            mdns.terminate()


if __name__ == '__main__':
    main()
