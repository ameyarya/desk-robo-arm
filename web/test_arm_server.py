"""Tests for the web app's control core and HTTP API, with a fake radio.

The fake bridge stands in for the USB bridge board and models the arm on the other end of the
radio: running our firmware (acks and replies), a single lost packet, the stock CyberBrick
firmware after a crash fallback (acks, never replies), switched off (no ack), and a slow link.
The real Robot class and the real ping() run on top of it.

Run: .venv/bin/python -m unittest discover web
"""
import http.client
import json
import os
import queue
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import arm_server  # noqa: E402
from arm_server import ArmControl, Refused, ping  # noqa: E402
from cyberbrick_bridge import Robot  # noqa: E402

MAC = 'aabbccddeeff'


class FakeBridge:
    """Bridge.send()/events/drain() as the real one, with the arm simulated in `mode`."""

    def __init__(self):
        self.mode = 'cbb'        # 'cbb' | 'stock' | 'off'
        self.drop = 0            # drop the next N packets (no ack)
        self.delay = 0.0         # seconds each send takes (a slow link)
        self.sent = []
        self.events = queue.Queue()
        self.lock = threading.Lock()

    def send(self, mac, msg):
        if self.delay:
            time.sleep(self.delay)
        with self.lock:
            self.sent.append(msg)
            if self.mode == 'off' or self.drop:
                self.drop = max(0, self.drop - 1)
                return False
        if self.mode == 'cbb' and msg.get('op') == 'ping':
            self.events.put({'ev': 'rx', 'from': mac,
                             'msg': {'op': 'ping', 'name': 'arm', 'version': '0.2.0', 'up_s': 5,
                                     'errors': 0, 'failsafe': 0, 'last_error': None, 'crash': None}})
        return True

    def drain(self):
        while not self.events.empty():
            self.events.get_nowait()

    def moves(self):
        """The motion commands sent (ignoring pings and keepalives)."""
        with self.lock:
            return [m for m in self.sent if m.get('op') not in ('ping', 'ka')]


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """An ArmControl wired to a FakeBridge and a real Robot, driven by hand."""

    def __init__(self, connect_ok=True):
        self.clock = Clock()
        self.bridge = FakeBridge()
        self.connect_ok = connect_ok

        def connect():
            if not self.connect_ok:
                raise arm_server.BridgeError("no robot named 'arm' answered")
            return self.bridge, Robot(self.bridge, MAC), {'mac': MAC, 'name': 'arm', 'version': '0.2.0'}

        self.control = ArmControl(connect, clock=self.clock,
                                  pinger=lambda b, m: ping(b, m, timeout=0.02))
        self.step()   # connects

    def step(self, dt=0.0):
        self.clock.t += dt
        self.control.tick()
        self.control.service()

    def pings(self, n):
        """Run n ping rounds."""
        for _ in range(n):
            self.step(arm_server.PING_S + 0.01)

    def armed(self):
        self.control.set_armed(True)
        return self


class LinkTest(unittest.TestCase):
    def test_one_lost_ping_does_not_disconnect_or_disarm(self):
        # the reported bug: a single missed radio ack flipped the page to offline and disarmed
        r = Rig().armed()
        r.bridge.drop = 1
        r.control.lease_until = r.clock.t + 60
        r.pings(1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'ok')
        self.assertTrue(s['armed'])
        r.pings(1)
        self.assertEqual(r.control.status()['link']['misses'], 0)

    def test_lost_after_consecutive_misses_then_recovers(self):
        r = Rig().armed()
        r.control.lease_until = r.clock.t + 60
        r.bridge.mode = 'off'
        r.pings(arm_server.LOST_AFTER - 1)
        self.assertEqual(r.control.status()['link']['state'], 'ok')
        r.pings(1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'lost')
        self.assertFalse(s['armed'])
        self.assertIn('switched on', s['problem'])
        r.bridge.mode = 'cbb'
        r.pings(1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'ok')
        self.assertIsNone(s['error'])
        r.control.set_armed(True)   # can arm again

    def test_acks_without_replies_are_recognised(self):
        r = Rig()
        r.bridge.mode = 'stock'
        r.pings(arm_server.LOST_AFTER)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'silent')
        self.assertIn('off and on', s['problem'])

    def test_arming_refused_with_the_reason_until_connected(self):
        r = Rig(connect_ok=False)
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True)
        self.assertIn('no robot', str(e.exception))
        self.assertTrue(r.control.status()['link']['bridge'])
        r.connect_ok = True
        r.step(arm_server.RECONNECT_S + 0.01)
        r.control.set_armed(True)

    def test_unplugged_bridge_is_reopened(self):
        r = Rig().armed()
        r.control.lease_until = r.clock.t + 60
        resets = []
        r.control.connect.reset = lambda: resets.append(1)
        unplugged = FakeBridge()
        unplugged.send = lambda mac, msg: (_ for _ in ()).throw(OSError('device disconnected'))
        r.control.bridge = r.control.robot.bridge = unplugged
        r.pings(arm_server.LOST_AFTER)
        s = r.control.status()
        self.assertFalse(s['armed'])
        self.assertEqual(s['link']['state'], 'connecting')
        self.assertFalse(s['link']['bridge'])
        self.assertEqual(resets, [1])
        r.step(arm_server.RECONNECT_S + 0.01)   # plugged back in: connect() works again
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'ok')
        self.assertTrue(s['link']['bridge'])
        r.control.set_armed(True)

    def test_ping_reports_health(self):
        r = Rig()
        r.pings(1)
        link = r.control.status()['link']
        self.assertEqual(link['health']['up_s'], 5)
        self.assertEqual(link['version'], '0.2.0')


class MotionTest(unittest.TestCase):
    def test_refuses_to_move_while_disarmed(self):
        r = Rig()
        with self.assertRaises(Refused):
            r.control.hold('base', 40)
        r.step()
        self.assertEqual(r.bridge.moves(), [])

    def test_hold_moves_once_then_stops_when_not_renewed(self):
        r = Rig().armed()
        r.control.hold('shoulder', 30)
        r.step()
        self.assertEqual(r.bridge.moves(), [{'op': 'servo_speed', 'ch': 1, 'v': 30}])
        r.control.hold('shoulder', 30)   # renewal: nothing new on the radio
        r.step(0.2)
        self.assertEqual(len(r.bridge.moves()), 1)
        r.step(arm_server.HOLD_S + 0.01)
        self.assertEqual(r.bridge.moves()[-1], {'op': 'servo_speed', 'ch': 1, 'v': 0})
        self.assertEqual(r.control.status()['running'], {'shoulder': 0})

    def test_joystick_speeds_are_rounded_and_only_changes_are_sent(self):
        r = Rig().armed()
        for speed in (21, 19, 22, 37, 38):
            r.control.hold('base', speed)
            r.step()
        # 21/19/22 all round to 20 (one command); 37 -> 35; 38 -> 40
        self.assertEqual([m['v'] for m in r.bridge.moves()], [20, 35, 40])
        self.assertEqual(r.control.status()['running'], {'base': 40})

    def test_speed_is_clamped(self):
        r = Rig().armed()
        r.control.hold('elbow', -500)
        r.step()
        self.assertEqual(r.bridge.moves()[-1]['v'], -arm_server.MAX_SPEED)

    def test_zero_hold_releases(self):
        r = Rig().armed()
        r.control.hold('elbow', 30)
        r.step()
        r.control.hold('elbow', 0)
        r.step()
        self.assertEqual(r.bridge.moves()[-1]['v'], 0)

    def test_failed_move_is_retried_until_acked(self):
        r = Rig().armed()
        r.bridge.drop = 2
        r.control.hold('base', 40)
        r.step()
        self.assertEqual(r.control.status()['running'], {})
        r.step(0.01)                       # within RETRY_S: no new attempt
        self.assertEqual(len(r.bridge.moves()), 1)
        r.step(arm_server.RETRY_S)
        r.step(arm_server.RETRY_S)
        self.assertEqual(r.control.status()['running'], {'base': 40})
        self.assertTrue(r.control.status()['armed'])

    def test_stop_disarms_and_is_retried_until_acked(self):
        r = Rig().armed()
        r.control.hold('base', 40)
        r.step()
        r.bridge.drop = 1
        s = r.control.stop('test')
        self.assertFalse(s['armed'])
        r.step()                            # stop not acked
        r.step(arm_server.RETRY_S + 0.01)   # retried
        self.assertEqual([m['op'] for m in r.bridge.moves()].count('stop'), 2)
        self.assertEqual(r.control.status()['running'], {})

    def test_lease_lapse_disarms_and_stops(self):
        r = Rig().armed()
        r.control.hold('base', 40)
        r.step()
        for _ in range(int(arm_server.LEASE_S / 0.2) + 2):   # keep holding, never renew the lease
            try:
                r.control.hold('base', 40)
            except Refused:
                break
            r.step(0.2)
        s = r.control.status()
        self.assertFalse(s['armed'])
        self.assertIn('lease lapsed', s['last'])
        self.assertEqual(r.bridge.moves()[-1], {'op': 'stop'})

    def test_renewing_after_a_server_side_disarm_says_why(self):
        r = Rig().armed()
        r.control.stop('page STOP')
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True, renew=True)
        self.assertIn('page STOP', str(e.exception))


class ConcurrencyTest(unittest.TestCase):
    def test_status_and_hold_never_wait_for_the_radio(self):
        r = Rig().armed()
        r.bridge.delay = 1.0                 # every radio call now takes a second
        r.control.hold('base', 40)
        worker = threading.Thread(target=r.control.service)
        worker.start()
        time.sleep(0.05)                     # the worker is inside a slow send
        t = time.monotonic()
        r.control.status()
        r.control.hold('base', 40)
        r.control.set_armed(True, renew=True)
        self.assertLess(time.monotonic() - t, 0.05)
        worker.join()


class HttpTest(unittest.TestCase):
    def setUp(self):
        self.rig = Rig()
        self.server = arm_server.QuietServer(('127.0.0.1', 0), arm_server.make_handler(self.rig.control, 'pw'))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def call(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        conn.request(method, path, body=None if body is None else json.dumps(body), headers=headers or {})
        res = conn.getresponse()
        data = res.read()
        conn.close()
        ctype = res.getheader('Content-Type', '')
        return res.status, (json.loads(data) if 'json' in ctype else data), res

    def test_page_and_status(self):
        status, page, _ = self.call('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn(b'Robo Arm', page)
        status, s, _ = self.call('GET', '/status')
        self.assertEqual(status, 200)
        self.assertEqual(s['link']['state'], 'ok')

    def test_refusals_come_back_as_409_with_reason_and_status(self):
        status, body, _ = self.call('POST', '/hold', {'joint': 'base', 'speed': 40})
        self.assertEqual(status, 409)
        self.assertEqual(body['error'], 'arm first')
        self.assertIn('link', body['status'])

    def test_arm_hold_stop_round_trip(self):
        self.assertEqual(self.call('POST', '/arm', {'armed': True})[0], 200)
        status, s, _ = self.call('POST', '/hold', {'joint': 'elbow', 'speed': -30})
        self.assertEqual(status, 200)
        self.assertEqual(s['wanted'], {'elbow': -30})
        status, s, _ = self.call('POST', '/stop', {'from': 'test'})
        self.assertFalse(s['armed'])
        self.assertEqual(s['last'], 'test')

    def test_bad_json_is_not_a_crash(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        conn.request('POST', '/stop', body='{not json', headers={'Content-Length': '9'})
        self.assertEqual(conn.getresponse().status, 200)

    def test_status_is_fast_while_the_radio_is_slow(self):
        self.call('POST', '/arm', {'armed': True})
        self.call('POST', '/hold', {'joint': 'base', 'speed': 40})
        self.rig.bridge.delay = 1.0
        worker = threading.Thread(target=self.rig.control.service)
        worker.start()
        time.sleep(0.05)
        t = time.monotonic()
        for _ in range(5):
            self.assertEqual(self.call('GET', '/status')[0], 200)
        self.assertLess(time.monotonic() - t, 0.5)
        worker.join()


class LanAuthTest(unittest.TestCase):
    """As seen from another device on the Wi-Fi (no address counts as local)."""

    def setUp(self):
        self.rig = Rig()
        handler = arm_server.make_handler(self.rig.control, 'pw', local=())
        self.server = arm_server.QuietServer(('127.0.0.1', 0), handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def request(self, method, path, body=b'', headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        conn.request(method, path, body=body, headers=headers or {})
        res = conn.getresponse()
        res.read()
        conn.close()
        return res

    def test_stop_works_without_login_but_arming_does_not(self):
        self.assertEqual(self.request('POST', '/stop', b'{}').status, 200)
        self.assertEqual(self.request('POST', '/arm', b'{"armed": true}').status, 401)
        self.assertFalse(self.rig.control.status()['armed'])

    def test_login_cookie_authorizes(self):
        res = self.request('POST', '/login', b'password=pw',
                           {'Content-Type': 'application/x-www-form-urlencoded'})
        self.assertEqual(res.status, 303)
        cookie = res.getheader('Set-Cookie').split(';')[0]
        self.assertEqual(self.request('POST', '/arm', b'{"armed": true}', {'Cookie': cookie}).status, 200)


if __name__ == '__main__':
    unittest.main()
