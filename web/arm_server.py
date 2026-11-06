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
import collections
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
RETRY_S = 0.1           # after a stop-a-joint command the arm didn't ack, try again this soon
RENEW_S = 0.2           # re-send a moving joint's speed this often (the arm's failsafe is 0.6 s)
STOP_RETRY_S = 0.15     # re-send an unacknowledged STOP this often
FAILSAFE_S = 0.6        # the arm firmware's failsafe (failsafe_ms in its cbb_config.json)
QUALITY_PINGS = 30      # signal quality is measured over this many pings
POOR_PCT = 80           # below this share of pings answered, the link counts as weak
PLACEMENT_TIP = ('USB 3 ports and hubs, and the Mac itself, jam 2.4 GHz radio: put the bridge on a USB '
                 'extension cable, away from the Mac and any USB 3 hub, with a clear line to the arm.')
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
    return at once. `service()` (the worker thread) does every radio call. Tests drive
    tick()/service() by hand with a fake clock and fake radio.

    How motion stays safe:
    - While a joint should move, its speed command is re-sent every RENEW_S. Nothing else is
      sent to keep it alive (no keepalives), so the moment the page lets go, disarms, or this
      process dies, the arm's own failsafe stops it within its failsafe time.
    - A joint that should stop gets a 0 command, re-sent until the arm acknowledges it.
    - STOP (disarm, lease lapse, page STOP, link lost) is sent whatever the link looks like and
      re-sent every STOP_RETRY_S until the arm acknowledges it; it is sent again when a lost
      link comes back. Until then the page shows the arm as possibly still moving.

    `connect()` returns (bridge, robot, info) or raises; it is retried every RECONNECT_S.
    `pinger(bridge, mac)` returns (reply, why) as ping() does.
    """

    def __init__(self, connect, joints=ARM_JOINTS, clock=time.monotonic, pinger=ping, wall=time.time,
                 log=print):
        self.connect = connect
        self.pinger = pinger
        self.joints = [j for j, (kind, _) in joints.items() if kind != 'angle']
        self.clock = clock
        self.wall = wall
        self.printer = log
        self.lock = threading.Lock()
        self.bridge = self.robot = None
        self.armed = False
        self.lease_until = 0.0
        self.holds = {}          # joint -> (speed, until): what the page asks for
        self.sent = {}           # joint -> last speed commanded (the arm may be doing this)
        self.renew_at = {}       # joint -> when to re-send its (non-zero) speed
        self.stop_pending = None  # reason, while a STOP waits for the arm's ack
        self.stop_attempts = 0
        self.stop_since = 0.0
        self.next_stop = 0.0
        self.link = {'state': 'connecting', 'bridge': False, 'mac': None, 'name': None, 'version': None,
                     'misses': 0, 'why': None, 'health': None, 'rtt_ms': None, 'last_reply_s': None}
        self.last_reply = None
        self.error = None
        self.last = 'nothing yet'
        self.next_ping = 0.0
        self.next_connect = 0.0
        self.events = collections.deque(maxlen=60)
        self.pings = collections.deque(maxlen=QUALITY_PINGS)   # (acked, replied) per ping
        self.poor = False
        self._log('info', 'server started; looking for the bridge board and the arm')

    # -- page requests (fast, never touch the radio) --------------------------

    def set_armed(self, armed, renew=False, who='page'):
        with self.lock:
            if armed:
                if self.link['state'] != 'ok':
                    raise Refused('cannot arm: ' + (self._link_problem() or 'the arm is not connected'))
                if renew and not self.armed:
                    raise Refused('disarmed: ' + self.last)
                if self.stop_pending:
                    raise Refused('cannot arm yet: still waiting for the arm to confirm the last STOP')
                if not self.armed:
                    self._log('info', f'armed by the {who}')
                    self.last = f'armed by the {who}'
                self.armed = True
                self.lease_until = self.clock() + LEASE_S
            elif self.armed:
                self._stop_locked(f'disarmed by the {who}')
            return self._status_locked()

    def hold(self, joint, speed):
        with self.lock:
            if joint not in self.joints:
                raise Refused(f'unknown joint {joint!r}')
            if not self.armed:
                raise Refused('not armed: press ARM first')
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

    def stop(self, reason='STOP pressed'):
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
                self._stop_locked('lease lapsed: the page stopped renewing it (closed, asleep, or lost '
                                  'its connection)')
            for joint, (_, until) in list(self.holds.items()):
                if now > until:
                    del self.holds[joint]

    def service(self):
        """All radio work: connect, ping, STOP and the joint speeds."""
        now = self.clock()
        if self.robot is None:
            if now >= self.next_connect:
                self.next_connect = now + RECONNECT_S
                self._connect()
            return
        if self.stop_pending:
            if now >= self.next_stop:
                self._send_stop(now)
        else:
            with self.lock:
                want = {j: s for j, (s, _) in self.holds.items()} if self.armed else {}
            for joint in self.joints:
                self._drive(joint, want.get(joint, 0), now)
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
                    self.error = f'internal error in the control loop: {error!r}'
                    self._log('error', self.error)
            time.sleep(0.03)

    # -- internals -----------------------------------------------------------

    def _send(self, msg):
        """One radio send. True if the arm acknowledged it."""
        try:
            return bool(self.bridge.send(self.robot.mac, msg))
        except Exception as error:
            with self.lock:
                self.error = f'bridge board error: {error}'
            return False

    def _drive(self, joint, speed, now):
        last = self.sent.get(joint, 0)
        if speed == last and (not speed or now < self.renew_at.get(joint, 0)):
            return
        acked = self._send(self.robot.command(joint, speed))
        with self.lock:
            if speed:
                # sent either way: the arm may have heard it even without an ack
                self.sent[joint] = speed
                self.renew_at[joint] = now + RENEW_S
                if speed != last:
                    self._log('info', f'{joint} {"starts" if not last else "changes to"} {speed:+d}%')
            elif acked:
                self.sent.pop(joint, None)
                self._log('info', f'{joint} stopped')
            else:
                self.renew_at[joint] = now + RETRY_S   # retry the 0 shortly
            if not acked and self.link['state'] == 'ok':
                self.error = f'{joint}: the arm did not acknowledge the command; retrying'
            elif acked and self.error and self.error.startswith(joint + ':'):
                self.error = None

    def _send_stop(self, now):
        self.stop_attempts += 1
        acked = self._send({'op': 'stop'})
        with self.lock:
            if acked:
                took = now - self.stop_since
                self._log('info', f'STOP confirmed by the arm ({self.stop_attempts} '
                                  f'{"try" if self.stop_attempts == 1 else "tries"}, {took * 1000:.0f} ms)')
                self.stop_pending = None
                self.sent.clear()
                self.renew_at.clear()
                if self.error and self.error.startswith('STOP'):
                    self.error = None
            else:
                self.next_stop = now + (STOP_RETRY_S if self.stop_attempts < 15 else 1.0)
                if self.stop_attempts in (1, 5) or self.stop_attempts % 25 == 0:
                    self._log('warn', f'STOP not acknowledged yet (try {self.stop_attempts}); retrying. '
                                      f'The arm stops on its own {FAILSAFE_S:.1f} s after the last motion command.')
                if self.link['state'] == 'ok':
                    self.error = f'STOP sent, waiting for the arm to confirm (try {self.stop_attempts})'

    def _connect(self):
        try:
            bridge, robot, info = self.connect()
        except Exception as error:
            with self.lock:
                bridge_ok = isinstance(error, BridgeError)
                why = str(error)
                if why != self.link['why']:
                    self._log('warn', why)
                self.link.update(state='connecting', why=why, bridge=bridge_ok)
                self.error = None
            return
        with self.lock:
            self.bridge, self.robot = bridge, robot
            self.link.update(state='ok', bridge=True, misses=0, why=None, **info)
            self.last_reply = self.clock()
            self.error = None
            self._log('info', f"found the arm '{info.get('name')}' ({info.get('mac')}), firmware {info.get('version')}")
            # whatever it was doing before this server started, make sure it is stopped
            self._stop_locked('server connected to the arm', quiet=True)

    def _ping(self):
        t = time.monotonic()
        try:
            reply, why = self.pinger(self.bridge, self.robot.mac)
        except Exception as error:   # the bridge itself failed (unplugged?)
            reply, why = None, f'bridge board error: {error}'
        with self.lock:
            now = self.clock()
            self.pings.append((reply is not None or why == 'acked but no reply', reply is not None))
            self._quality_locked()
            if reply is not None:
                was = self.link['state']
                old = self.link.get('health') or {}
                health = {k: reply.get(k) for k in ('up_s', 'errors', 'failsafe', 'last_error', 'crash', 'mem')}
                self.link.update(state='ok', misses=0, why=None, version=reply.get('version'),
                                 rtt_ms=round((time.monotonic() - t) * 1000), health=health)
                self.last_reply = now
                if was != 'ok':
                    restarted = old.get('up_s') is not None and (health.get('up_s') or 0) < old['up_s']
                    self._log('info', f'the arm is answering again (firmware {reply.get("version")}, '
                                      f'up {health.get("up_s")} s: '
                                      + ('it restarted)' if restarted else 'it kept running, so only its '
                                         'replies were lost)'))
                    self._stop_locked('link came back', quiet=True)   # make sure it is stopped
                if old.get('up_s') is not None and (health.get('up_s') or 0) < old['up_s']:
                    self._log('warn', f'the arm restarted (uptime went from {old["up_s"]} s to {health.get("up_s")} s)')
                if (health.get('failsafe') or 0) > (old.get('failsafe') or 0) and old:
                    self._log('warn', 'the arm\'s failsafe stopped it: it got no motion command for '
                                      f'{FAILSAFE_S:.1f} s while moving')
                if (health.get('errors') or 0) > (old.get('errors') or 0) and old:
                    self._log('warn', f'the arm reported a firmware error: {health.get("last_error")}')
                if health.get('crash') and health.get('crash') != old.get('crash'):
                    self._log('error', f'the arm\'s firmware crashed earlier: {health["crash"]}')
                if self.error and self.error.startswith('arm '):
                    self.error = None
                return
            self.link['misses'] += 1
            self.link['why'] = why
            if self.link['misses'] >= LOST_AFTER and self.link['state'] == 'ok':
                self.link['state'] = 'silent' if why == 'acked but no reply' else 'lost'
                self._log('error', 'arm lost: ' + self._link_problem())
                if self.armed or self.holds or self.sent:
                    self._stop_locked('arm lost')
            if why.startswith('bridge'):
                self.link['bridge'] = False
                if self.link['misses'] >= LOST_AFTER:
                    self._drop_bridge_locked()

    def _quality(self):
        n = len(self.pings)
        if not n:
            return None
        return {'pings': n, 'acked_pct': round(100 * sum(a for a, _ in self.pings) / n),
                'replied_pct': round(100 * sum(r for _, r in self.pings) / n)}

    def _quality_locked(self):
        q = self._quality()
        if not q or q['pings'] < 10:
            return
        poor = q['replied_pct'] < POOR_PCT
        if poor and not self.poor:
            self._log('warn', f"weak radio link: of the last {q['pings']} pings {q['acked_pct']}% were "
                              f"acknowledged and {q['replied_pct']}% answered. " + PLACEMENT_TIP)
        elif self.poor and q['replied_pct'] >= POOR_PCT + 10:
            self._log('info', f"radio link good again ({q['replied_pct']}% of the last {q['pings']} pings answered)")
        self.poor = poor if poor or q['replied_pct'] >= POOR_PCT + 10 else self.poor

    def _drop_bridge_locked(self):
        """Forget a failed bridge (unplugged, or its port died) so the worker reopens it."""
        self._log('error', 'the bridge board stopped responding on USB (unplugged?); reconnecting')
        self.robot = self.bridge = None
        self.armed = False
        self.holds.clear()
        self.sent.clear()
        self.stop_pending = None
        self.link.update(state='connecting', bridge=False, why='bridge board lost on USB; reconnecting')
        reset = getattr(self.connect, 'reset', None)
        if reset:
            try:
                reset()
            except Exception:
                pass

    def _quality_text(self):
        q = self._quality()
        if not q:
            return ''
        return f"Signal over the last {q['pings']} pings: {q['acked_pct']}% acknowledged, {q['replied_pct']}% answered. "

    def _ago(self):
        if self.last_reply is None:
            return 'no reply yet'
        return f'last reply {self.clock() - self.last_reply:.0f} s ago'

    def _link_problem(self):
        state = self.link['state']
        if state == 'silent':
            return ("the arm acknowledges the radio (so it is powered and hears the Mac), but none of its "
                    f"replies arrived for {self.link['misses']} pings in a row ({self._ago()}). Either its "
                    "replies are lost on the way back (weak signal: low battery, distance, or the bridge's "
                    "position), or its firmware has frozen. Moving is blocked until it replies again. "
                    + self._quality_text() + PLACEMENT_TIP + " If its uptime restarts when it comes back, "
                    "it had frozen: switch it off and on.")
        if state == 'lost':
            return (f"no radio acknowledgement from the arm for {self.link['misses']} pings in a row "
                    f"({self._ago()}). It is switched off, out of battery, or out of range. "
                    + self._quality_text())
        if state == 'connecting':
            return self.link['why'] or 'looking for the bridge board and the arm'
        return None

    def _stop_locked(self, reason, quiet=False):
        was_armed = self.armed
        self.armed = False
        self.holds.clear()
        if not self.stop_pending:
            self.stop_attempts = 0
            self.stop_since = self.clock()
        self.stop_pending = reason
        self.next_stop = 0.0
        if not quiet or was_armed:
            self.last = reason
            self._log('warn' if reason.startswith(('arm lost', 'lease')) else 'info',
                      f'STOP: {reason}')

    def _log(self, level, msg):
        entry = {'t': time.strftime('%H:%M:%S', time.localtime(self.wall())), 'level': level, 'msg': msg}
        self.events.append(entry)
        try:
            self.printer(f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.wall()))} "
                         f"{level.upper():5} {msg}", flush=True)
        except Exception:
            pass

    def _status_locked(self):
        link = dict(self.link)
        link['last_reply_s'] = None if self.last_reply is None else round(self.clock() - self.last_reply, 1)
        link['quality'] = self._quality()
        link['poor'] = self.poor
        moving = {j: s for j, s in self.sent.items() if s}
        return {'armed': self.armed, 'joints': self.joints,
                'running': moving,               # what the arm may be doing right now
                'wanted': {j: s for j, (s, _) in self.holds.items()},
                'stopping': self.stop_pending, 'stop_attempts': self.stop_attempts if self.stop_pending else 0,
                'link': link, 'problem': self._link_problem(), 'error': self.error, 'last': self.last,
                'events': list(self.events)[-25:], 'max_speed': MAX_SPEED}


ARM_MAC_FILE = ROOT / '.arm-mac'   # the arm's radio MAC from the last time it was found (gitignored)


def _remembered_mac():
    try:
        return ARM_MAC_FILE.read_text().strip() or None
    except OSError:
        return None


def _remember_mac(mac):
    try:
        if _remembered_mac() != mac:
            ARM_MAC_FILE.write_text(mac + '\n')
    except OSError:
        pass


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
        try:
            # a direct ping is retried by the radio, a broadcast scan is not: try the arm we
            # found last time first
            known = mac or _remembered_mac()
            found = {}
            if known:
                reply, _ = ping(bridge, known, timeout=0.5)
                if reply is not None:
                    found = {known: reply}
            if not found:
                found = bridge.scan(1.0)
        except (BridgeError, OSError, ValueError) as error:
            # the USB connection died (unplugged, or the board reset): reopen it next time
            reset()
            raise OSError(f'bridge board USB error ({error}); reopening it') from None
        for robot_mac, info in found.items():
            if (mac and robot_mac == mac) or (not mac and info.get('name') == name):
                if info.get('version') == 'loader':
                    raise BridgeError(f'{robot_mac} is still running the install loader; run tools/finish_install.py')
                robot = Robot(bridge, robot_mac, keepalive=False)
                _remember_mac(robot_mac)
                return bridge, robot, {'mac': robot_mac, 'name': info.get('name'), 'version': info.get('version')}
        raise BridgeError(f'no robot named {name!r} answered (is it on? if it was just restarted, give it 5 s)'
                          + (f'; saw {sorted(found)}' if found else ''))

    def reset():
        bridge, state['bridge'] = state['bridge'], None
        if bridge is not None:
            try:
                bridge.close()
            except Exception:
                pass

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


def make_handler(control, password, local=('127.0.0.1', '::1'), sim=None):
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
            if self.path == '/sim' and sim is not None:   # --simulate only: break the simulated radio
                for key in ('mode', 'drop', 'ack_loss'):
                    if key in body:
                        setattr(sim, key, body[key])
                return self.reply(200, {'mode': sim.mode, 'drop': sim.drop, 'ack_loss': sim.ack_loss,
                                        'arm_moving': {f'{k[0]}{k[1]}': v for k, v in sim.arm_moving().items()}})
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

    sim = None
    if args.simulate:
        from sim import sim_connect
        connect = sim_connect()
        sim = connect.bridge
    else:
        connect = make_connect(args.usb, args.robot, args.mac)
    control = ArmControl(connect)
    threading.Thread(target=control.run, daemon=True).start()
    server = QuietServer((args.bind, args.port), make_handler(control, dashboard_password(), sim=sim))
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
