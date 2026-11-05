"""Robo Arm web app: drive the L-ONE arm's joints from a browser.

Run: .venv/bin/python web/arm_server.py
Then open http://robo-arm.local:8020/ from any device on the home Wi-Fi (advertised over mDNS),
or http://localhost:8020/ on this Mac. Other devices log in with the password in
web/.dashboard-password (created on first run, gitignored); this Mac needs no login. STOP works
without logging in. The page is web/arm.html.

- Owns the bridge board's USB port (the drive server of the rover project can't run at the same
  time) and finds the arm over the radio by name.
- Arm: the page holds a 3 s lease and renews it every second; when it lapses, or the page
  closes, everything stops and it disarms.
- Hold-to-move: while armed, the page renews a joint's hold every 150 ms; 0.35 s after the last
  renewal that joint stops. The arm's own failsafe stops it 0.6 s after the radio goes quiet.
"""
import argparse
import hashlib
import hmac
import json
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
MAX_SPEED = 80          # page sliders go up to this; the server clamps to it too
LINK_CHECK_S = 2.0
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


class ArmControl:
    """Arm lease, per-joint holds and the radio link, independent of HTTP (tested with a fake robot).

    `connect` returns (bridge, robot) or raises; it is retried until it works.
    """

    def __init__(self, connect, joints=ARM_JOINTS, clock=time.monotonic):
        self.connect = connect
        self.joints = [j for j, (kind, _) in joints.items() if kind != 'angle']
        self.clock = clock
        self.lock = threading.Lock()
        self.bridge = self.robot = None
        self.armed = False
        self.lease_until = 0.0
        self.holds = {}        # joint -> (speed, until)
        self.running = {}      # joint -> speed last sent to the arm
        self.link = {'bridge': False, 'arm': False, 'mac': None, 'name': None, 'version': None}
        self.error = None
        self.last = 'nothing yet'
        self.next_link_check = 0.0

    # -- page requests -------------------------------------------------------

    def set_armed(self, armed, renew=False):
        with self.lock:
            if armed and not self.link['arm']:
                raise Refused('the arm is not connected')
            if armed:
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
            speed = max(-MAX_SPEED, min(MAX_SPEED, int(speed)))
            self.holds[joint] = (speed, self.clock() + HOLD_S)
            self._apply_locked()
            return self._status_locked()

    def release(self, joint):
        with self.lock:
            self.holds.pop(joint, None)
            self._apply_locked()
            return self._status_locked()

    def stop(self, reason='STOP'):
        with self.lock:
            self._stop_locked(reason)
            return self._status_locked()

    def status(self):
        with self.lock:
            return self._status_locked()

    # -- background ----------------------------------------------------------

    def tick(self):
        """Expire the lease and holds, keep the link checked. Called every 50 ms."""
        with self.lock:
            now = self.clock()
            if self.armed and now > self.lease_until:
                self._stop_locked('lease lapsed (page closed or lost connection)')
            for joint, (_, until) in list(self.holds.items()):
                if now > until:
                    del self.holds[joint]
            self._apply_locked()
            if now >= self.next_link_check:
                self.next_link_check = now + LINK_CHECK_S
                self._check_link_locked()

    def run(self):
        while True:
            try:
                self.tick()
            except Exception as error:   # keep the watchdog alive whatever happens
                self.error = f'internal: {error}'
            time.sleep(0.05)

    # -- internals (lock held) -----------------------------------------------

    def _apply_locked(self):
        want = {j: s for j, (s, _) in self.holds.items()}
        for joint in self.joints:
            speed = want.get(joint, 0)
            if self.running.get(joint, 0) == speed:
                continue
            try:
                self.robot.move(joint, speed)
                self.running[joint] = speed
                self.error = None
                if speed:
                    self.last = f'{joint} {speed:+d}%'
            except Exception as error:
                self._lost_locked(error)
                return

    def _stop_locked(self, reason):
        self.armed = False
        self.holds.clear()
        self.last = reason
        if self.robot is not None:
            try:
                self.robot.stop()
            except Exception as error:
                self._lost_locked(error)
        self.running = {}

    def _lost_locked(self, error):
        self.error = str(error)
        self.link['arm'] = False
        self.armed = False
        self.holds.clear()
        self.running = {}

    def _check_link_locked(self):
        if self.robot is None:
            try:
                self.bridge, self.robot, info = self.connect()
                self.link.update(bridge=True, arm=True, **info)
                self.error = None
            except Exception as error:
                self.link['arm'] = False
                self.error = str(error)
            return
        try:
            ok = self.bridge.send(self.robot.mac, {'op': 'ka'})
        except Exception as error:
            self.link['bridge'] = False
            self._lost_locked(f'bridge: {error}')
            return
        self.link['bridge'] = True
        if ok and not self.link['arm']:
            self.error = None
        self.link['arm'] = bool(ok)
        if not ok:
            self._lost_locked('the arm stopped answering (off, or out of range?)')

    def _status_locked(self):
        return {'armed': self.armed, 'joints': self.joints, 'running': dict(self.running),
                'link': dict(self.link), 'error': self.error, 'last': self.last, 'max_speed': MAX_SPEED}


def make_connect(port, name, mac):
    """Open the bridge once, then find the arm by MAC or by name over the radio."""
    state = {'bridge': None}

    def connect():
        if state['bridge'] is None:
            state['bridge'] = Bridge(port)
        bridge = state['bridge']
        found = bridge.scan(1.0)
        for robot_mac, info in found.items():
            if (mac and robot_mac == mac) or (not mac and info.get('name') == name):
                if info.get('version') == 'loader':
                    raise BridgeError(f'{robot_mac} is still running the install loader; run tools/finish_install.py')
                robot = Robot(bridge, robot_mac)
                return bridge, robot, {'mac': robot_mac, 'name': info.get('name'), 'version': info.get('version')}
        raise BridgeError(f'no robot named {name!r} answered' + (f' (saw {sorted(found)})' if found else ''))

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

    def handle_error(self, request, client_address):
        if not isinstance(sys.exc_info()[1], (ConnectionResetError, BrokenPipeError)):
            super().handle_error(request, client_address)


def make_handler(control, password):
    token = session_token(password)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def authorized(self):
            if self.client_address[0] in ('127.0.0.1', '::1'):
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
            return json.loads(self.rfile.read(size) or b'{}') if size else {}

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
                return self.reply(200, control.stop(self.body().get('from', 'STOP')))
            if not self.authorized():
                return self.reply(401, {'error': 'log in first'})
            try:
                body = self.body()
                if self.path == '/arm':
                    return self.reply(200, control.set_armed(bool(body.get('armed')), bool(body.get('renew'))))
                if self.path == '/hold':
                    return self.reply(200, control.hold(body.get('joint'), body.get('speed', 0)))
                if self.path == '/release':
                    return self.reply(200, control.release(body.get('joint')))
            except Refused as error:
                return self.reply(409, {'error': str(error)})
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
    args = parser.parse_args()

    control = ArmControl(make_connect(args.usb, args.robot, args.mac))
    threading.Thread(target=control.run, daemon=True).start()
    server = QuietServer((args.bind, args.port), make_handler(control, dashboard_password()))
    mdns = None
    if args.bind not in ('127.0.0.1', 'localhost'):
        ip = lan_ip()
        mdns = advertise(args.name, args.port, ip)
        print(f'Robo Arm on http://{args.name}.local:{args.port}/ ({ip}) and http://localhost:{args.port}/'
              f'  (LAN login password in web/{PASSWORD_FILE.name})', flush=True)
    else:
        print(f'Robo Arm on http://localhost:{args.port}/', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        control.stop('server stopped')
        if mdns:
            mdns.terminate()


if __name__ == '__main__':
    main()
