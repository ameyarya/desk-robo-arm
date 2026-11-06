"""Tests for the web app's control core and HTTP API, against the simulated arm in sim.py.

The simulated arm runs its joints and its own failsafe, so the tests check what the arm actually
does, not just what was sent. The real Robot class and the real ping() run on top of it.

Run: .venv/bin/python -m unittest discover web
"""
import http.client
import json
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import arm_server  # noqa: E402
from arm_server import ArmControl, Refused, ping  # noqa: E402
from sim import SimBridge, sim_connect  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """An ArmControl wired to a simulated arm, stepped by hand with a fake clock."""

    def __init__(self, connect_ok=True, failsafe_counts='motion'):
        self.clock = Clock()
        self.bridge = SimBridge(clock=self.clock, failsafe_counts=failsafe_counts)
        self.connect_ok = connect_ok
        self.connect_error = arm_server.BridgeError("no robot named 'arm' answered")
        real = sim_connect(self.bridge)
        self.log = []

        def connect():
            if not self.connect_ok:
                raise self.connect_error
            return real()

        self.control = ArmControl(connect, clock=self.clock, pinger=lambda b, m: ping(b, m, timeout=0.02),
                                  log=lambda line, **kw: self.log.append(line))
        self.step()   # connects
        self.step()   # sends the "make sure it's stopped" STOP

    def step(self, dt=0.0):
        self.clock.t += dt
        self.control.tick()
        self.control.service()

    def run(self, seconds, dt=0.03, renew_lease=True, hold=None):
        """Run the worker for `seconds`, like the real loop; optionally keep the page's lease and a hold."""
        end = self.clock.t + seconds
        while self.clock.t < end:
            if renew_lease and self.control.armed:
                self.control.set_armed(True, renew=True)
            if hold and self.control.armed:
                self.control.hold(*hold)
            self.step(dt)

    def arm(self):
        self.control.set_armed(True)
        return self

    def events(self):
        return [e['msg'] for e in self.control.events]


class RunawayTest(unittest.TestCase):
    """The arm must stop whatever happens to the radio. (It once kept spinning after a disarm.)"""

    def test_no_keepalive_packets_are_ever_sent(self):
        r = Rig().arm()
        r.run(1.0, hold=('base', 40))
        r.run(1.0)
        self.assertNotIn('ka', [m['op'] for m in r.bridge.sent])

    def test_release_stops_the_arm_even_if_every_later_packet_is_lost(self):
        r = Rig().arm()
        r.run(0.5, hold=('base', 40))
        self.assertTrue(r.bridge.arm_moving())
        r.bridge.mode = 'off'                       # radio dead from here on
        r.run(arm_server.FAILSAFE_S + 0.1)
        self.assertEqual(r.bridge.arm_moving(), {})
        self.assertGreaterEqual(r.bridge.failsafe_trips, 1)

    def test_disarm_with_lost_acks_stops_the_arm_and_keeps_sending_stop(self):
        # the reported bug: STOP's ack lost, packets still arriving, arm kept going
        r = Rig().arm()
        r.run(0.5, hold=('elbow', 30))
        r.bridge.ack_loss = 10_000                  # everything arrives, no ack ever comes back
        n = len(r.bridge.moves())
        r.control.stop('page STOP')
        r.run(2.0)
        self.assertEqual(r.bridge.arm_moving(), {})
        ops = [m['op'] for m in r.bridge.moves()[n:]]
        self.assertEqual(set(ops), {'stop'})        # only STOPs after the disarm, no motion
        s = r.control.status()
        self.assertFalse(s['armed'])
        self.assertTrue(s['stopping'])
        r.bridge.ack_loss = 0
        r.run(0.3)
        self.assertIsNone(r.control.status()['stopping'])
        self.assertTrue(any('STOP confirmed by the receiver' in e for e in r.events()))

    def test_old_firmware_that_counts_pings_still_stops(self):
        # firmware before 0.2.2 refreshed its failsafe on any packet, pings included
        r = Rig(failsafe_counts='any').arm()
        r.run(0.5, hold=('shoulder', 30))
        r.bridge.drop = 10_000                      # the release and STOP never arrive
        r.control.stop('page STOP')
        r.bridge.drop = 0
        r.bridge.ack_loss = 0
        # pings still arrive (one per second) but STOPs are being lost: model it by losing all
        # stop commands and keeping pings
        orig = r.bridge._arrive
        r.bridge._arrive = lambda m: None if m.get('op') == 'stop' else orig(m)
        r.run(2.0)
        self.assertEqual(r.bridge.arm_moving(), {})

    def test_link_lost_while_moving_sends_stop_and_again_when_it_returns(self):
        r = Rig().arm()
        r.run(0.5, hold=('base', 40))
        r.bridge.mode = 'off'
        r.run(arm_server.PING_S * (arm_server.LOST_AFTER + 1), hold=('base', 40))
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'lost')
        self.assertFalse(s['armed'])
        stops_while_lost = sum(m['op'] == 'stop' for m in r.bridge.sent)
        self.assertGreater(stops_while_lost, 2)     # kept trying although the link looked dead
        r.bridge.mode = 'cbb'
        r.run(arm_server.PING_S + 0.2)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'ok')
        self.assertIsNone(s['stopping'])            # a STOP got through after the link came back
        self.assertEqual(r.bridge.arm_moving(), {})
        self.assertTrue(any('receiver is answering again' in e for e in r.events()))

    def test_lease_lapse_stops_the_arm(self):
        r = Rig().arm()
        r.run(0.5, hold=('base', 40))
        r.run(arm_server.LEASE_S + 0.5, renew_lease=False, hold=('base', 40))   # page stops renewing
        self.assertFalse(r.control.status()['armed'])
        self.assertEqual(r.bridge.arm_moving(), {})
        self.assertTrue(any('lease lapsed' in e for e in r.events()))

    def test_cannot_arm_until_stop_is_confirmed(self):
        r = Rig().arm()
        r.bridge.ack_loss = 10_000
        r.control.stop('test')
        r.step()
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True)
        self.assertIn('confirm the last STOP', str(e.exception))
        r.bridge.ack_loss = 0
        r.run(0.3)
        r.control.set_armed(True)


class LinkTest(unittest.TestCase):
    def test_one_lost_ping_does_not_disconnect_or_disarm(self):
        r = Rig().arm()
        r.bridge.drop = 1
        r.run(arm_server.PING_S * 2 + 0.1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'ok')
        self.assertTrue(s['armed'])

    def test_lost_after_consecutive_misses_with_a_precise_reason(self):
        r = Rig().arm()
        r.bridge.mode = 'off'
        r.run(arm_server.PING_S * (arm_server.LOST_AFTER - 1) + 0.1)
        self.assertEqual(r.control.status()['link']['state'], 'ok')
        r.run(arm_server.PING_S + 0.1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'lost')
        self.assertFalse(s['armed'])
        self.assertIn('no radio acknowledgement from the receiver', s['problem'])
        self.assertIn('pings in a row', s['problem'])
        self.assertIn('switched off, out of battery, or out of range', s['problem'])
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True)
        self.assertIn('cannot arm', str(e.exception))

    def test_acks_without_replies_are_reported_as_stopped_firmware(self):
        r = Rig()
        r.bridge.mode = 'stock'
        r.run(arm_server.PING_S * arm_server.LOST_AFTER + 0.1)
        s = r.control.status()
        self.assertEqual(s['link']['state'], 'silent')
        self.assertIn('acknowledges the radio', s['problem'])
        self.assertIn('replies are lost on the way back', s['problem'])

    def test_arming_refused_with_the_reason_until_connected(self):
        r = Rig(connect_ok=False)
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True)
        self.assertIn('no robot', str(e.exception))
        self.assertTrue(r.control.status()['link']['bridge'])
        r.connect_ok = True
        r.run(arm_server.RECONNECT_S + 0.1)
        r.control.set_armed(True)

    def test_unplugged_bridge_is_reopened(self):
        r = Rig().arm()
        resets = []
        r.control.connect.reset = lambda: resets.append(1)
        r.bridge.send = lambda mac, msg: (_ for _ in ()).throw(OSError('device disconnected'))
        r.connect_ok = False                         # and it can't be reopened while unplugged
        r.connect_error = OSError('bridge board not found on USB')
        r.run(arm_server.PING_S * arm_server.LOST_AFTER + 0.1)
        s = r.control.status()
        self.assertFalse(s['armed'])
        self.assertEqual(s['link']['state'], 'connecting')
        self.assertFalse(s['link']['bridge'])
        self.assertEqual(resets, [1])
        del r.bridge.send                            # plugged back in
        r.connect_ok = True
        r.run(arm_server.RECONNECT_S + 0.5)
        self.assertEqual(r.control.status()['link']['state'], 'ok')

    def test_usb_error_while_searching_reopens_the_bridge(self):
        # the bridge dropped off USB while the server was still looking for the arm
        opened, closed = [], []

        class DeadThenAlive:
            def __init__(self, port):
                opened.append(self)
                self.dead = len(opened) == 1

            def scan(self, wait):
                if self.dead:
                    raise OSError('write failed: [Errno 6] Device not configured')
                return {'aabbccddeeff': {'name': 'arm', 'version': '0.2.2'}}

            def close(self):
                closed.append(self)

        real, real_file = arm_server.Bridge, arm_server.ARM_MAC_FILE
        arm_server.Bridge = DeadThenAlive
        arm_server.ARM_MAC_FILE = arm_server.Path(os.devnull + '.missing')   # no remembered arm
        try:
            connect = arm_server.make_connect(None, 'arm', None)
            with self.assertRaises(OSError) as e:
                connect()
            self.assertIn('transmitter USB error', str(e.exception))
            self.assertEqual(len(closed), 1)
            bridge, robot, info = connect()       # reopened, and finds the arm
            self.assertEqual(len(opened), 2)
            self.assertEqual(info['version'], '0.2.2')
        finally:
            arm_server.Bridge, arm_server.ARM_MAC_FILE = real, real_file

    def test_signal_quality_is_measured_and_a_weak_link_is_flagged(self):
        r = Rig()
        r.run(arm_server.PING_S * 12)
        q = r.control.status()['link']['quality']
        self.assertEqual((q['acked_pct'], q['replied_pct']), (100, 100))
        self.assertFalse(r.control.status()['link']['poor'])
        r.bridge.mode = 'stock'                      # acks, no replies
        r.run(arm_server.PING_S * 8)
        s = r.control.status()
        self.assertTrue(s['link']['poor'])
        self.assertLess(s['link']['quality']['replied_pct'], arm_server.POOR_PCT)
        self.assertTrue(any('weak radio link' in e and 'USB extension' in e for e in r.events()))
        self.assertIn('acknowledged', s['problem'])  # the lost-link message quotes the numbers
        r.bridge.mode = 'cbb'
        r.run(arm_server.PING_S * 30)
        self.assertFalse(r.control.status()['link']['poor'])
        self.assertTrue(any('radio link good again' in e for e in r.events()))

    def test_arm_restart_and_failsafe_are_logged(self):
        r = Rig().arm()
        r.run(0.3, hold=('base', 40))
        r.bridge.mode = 'off'                         # stop renewals reaching it: failsafe trips
        r.run(0.8)
        r.bridge.mode = 'cbb'
        r.run(arm_server.PING_S + 0.1)
        self.assertTrue(any("failsafe stopped the arm" in e for e in r.events()))
        r.bridge.started = r.clock.t                 # uptime goes back to 0: a restart
        r.run(arm_server.PING_S + 0.1)
        self.assertTrue(any('the receiver restarted' in e for e in r.events()))


class MotionTest(unittest.TestCase):
    def test_refuses_to_move_while_disarmed(self):
        r = Rig()
        with self.assertRaises(Refused) as e:
            r.control.hold('base', 40)
        self.assertIn('press ARM', str(e.exception))
        r.run(0.3)
        self.assertEqual(r.bridge.arm_moving(), {})

    def test_hold_runs_the_joint_and_renews_it_then_stops(self):
        r = Rig().arm()
        n0 = len(r.bridge.moves())
        r.run(1.0, hold=('shoulder', 30))
        self.assertEqual(r.bridge.arm_moving(), {('servo_speed', 1): 30})
        renewals = [m for m in r.bridge.moves()[n0:] if m == {'op': 'servo_speed', 'ch': 1, 'v': 30}]
        self.assertGreaterEqual(len(renewals), int(1.0 / arm_server.RENEW_S) - 1)
        r.run(arm_server.HOLD_S + 0.1)               # page lets go
        self.assertEqual(r.bridge.arm_moving(), {})
        self.assertEqual(r.bridge.moves()[-1], {'op': 'servo_speed', 'ch': 1, 'v': 0})
        n1 = len(r.bridge.moves())
        r.run(1.0)                                   # nothing more is sent while stopped
        self.assertEqual(len(r.bridge.moves()), n1)

    def test_joystick_speeds_are_rounded(self):
        r = Rig().arm()
        for speed, want in ((21, 20), (19, 20), (37, 35), (38, 40)):
            r.control.hold('base', speed)
            r.step()
            self.assertEqual(r.control.status()['running'], {'base': want})

    def test_speed_is_clamped(self):
        r = Rig().arm()
        r.control.hold('elbow', -500)
        r.step()
        self.assertEqual(r.bridge.arm_moving(), {('servo_speed', 2): -arm_server.MAX_SPEED})

    def test_a_lost_zero_command_is_retried(self):
        r = Rig().arm()
        r.run(0.3, hold=('base', 40))
        r.bridge.drop = 2
        r.control.release('base')
        r.run(0.5)
        self.assertEqual(r.bridge.arm_moving(), {})
        self.assertEqual(r.control.status()['running'], {})

    def test_renewing_after_a_server_side_disarm_says_why(self):
        r = Rig().arm()
        r.control.stop('page STOP')
        with self.assertRaises(Refused) as e:
            r.control.set_armed(True, renew=True)
        self.assertIn('page STOP', str(e.exception))

    def test_events_describe_what_happened(self):
        r = Rig().arm()
        r.run(0.3, hold=('elbow', -30))
        r.run(0.5)
        r.control.stop('STOP pressed')
        r.run(0.3)
        ev = r.events()
        for expected in ('found the receiver', 'armed by the page', 'elbow starts -30%', 'elbow stopped',
                         'STOP: STOP pressed', 'STOP confirmed by the receiver'):
            self.assertTrue(any(expected in e for e in ev), f'{expected!r} not in {ev}')
        self.assertTrue(any('STOP: STOP pressed' in line for line in r.log))   # also in the log file


class ConcurrencyTest(unittest.TestCase):
    def test_requests_never_wait_for_the_radio(self):
        r = Rig().arm()
        r.bridge.delay = 1.0
        r.control.hold('base', 40)
        worker = threading.Thread(target=r.control.service)
        worker.start()
        time.sleep(0.05)
        t = time.monotonic()
        r.control.status()
        r.control.hold('base', 40)
        r.control.set_armed(True, renew=True)
        r.control.stop('test')
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

    def call(self, method, path, body=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        conn.request(method, path, body=None if body is None else json.dumps(body))
        res = conn.getresponse()
        data = res.read()
        conn.close()
        return res.status, (json.loads(data) if 'json' in res.getheader('Content-Type', '') else data)

    def test_page_and_status(self):
        status, page = self.call('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn(b'Robo Arm', page)
        status, s = self.call('GET', '/status')
        self.assertEqual(s['link']['state'], 'ok')
        self.assertIn('events', s)

    def test_refusals_come_back_as_409_with_reason_and_status(self):
        status, body = self.call('POST', '/hold', {'joint': 'base', 'speed': 40})
        self.assertEqual(status, 409)
        self.assertIn('press ARM', body['error'])
        self.assertIn('link', body['status'])

    def test_arm_hold_stop_round_trip(self):
        self.assertEqual(self.call('POST', '/arm', {'armed': True})[0], 200)
        status, s = self.call('POST', '/hold', {'joint': 'elbow', 'speed': -30})
        self.assertEqual(s['wanted'], {'elbow': -30})
        status, s = self.call('POST', '/stop', {'from': 'test'})
        self.assertFalse(s['armed'])
        self.assertEqual(s['last'], 'test')

    def test_bad_json_is_not_a_crash(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        conn.request('POST', '/stop', body='{not json', headers={'Content-Length': '9'})
        self.assertEqual(conn.getresponse().status, 200)


class LanAuthTest(unittest.TestCase):
    """As seen from another device on the Wi-Fi (no address counts as local)."""

    def setUp(self):
        self.rig = Rig()
        self.server = arm_server.QuietServer(('127.0.0.1', 0),
                                             arm_server.make_handler(self.rig.control, 'pw', local=()))
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

    def test_login_cookie_authorizes(self):
        res = self.request('POST', '/login', b'password=pw', {'Content-Type': 'application/x-www-form-urlencoded'})
        self.assertEqual(res.status, 303)
        cookie = res.getheader('Set-Cookie').split(';')[0]
        self.assertEqual(self.request('POST', '/arm', b'{"armed": true}', {'Cookie': cookie}).status, 200)


if __name__ == '__main__':
    unittest.main()
