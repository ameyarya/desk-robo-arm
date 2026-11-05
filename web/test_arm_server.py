"""ArmControl with a fake robot: lease expiry, refusals, hold watchdog, STOP, link loss.

Run: .venv/bin/python -m unittest web/test_arm_server.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import arm_server  # noqa: E402
from arm_server import ArmControl, Refused  # noqa: E402


class FakeRobot:
    mac = 'aabbccddeeff'

    def __init__(self):
        self.calls = []
        self.fail = False

    def move(self, joint, speed):
        if self.fail:
            raise ConnectionError('no ack')
        self.calls.append((joint, speed))

    def stop(self):
        self.calls.append('stop')


class FakeBridge:
    def __init__(self):
        self.ack = True

    def send(self, mac, msg):
        return self.ack


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make():
    clock, robot, bridge = Clock(), FakeRobot(), FakeBridge()
    c = ArmControl(lambda: (bridge, robot, {'mac': robot.mac, 'name': 'arm', 'version': '0.1.0'}), clock=clock)
    c.tick()   # connects
    return c, clock, robot, bridge


class ArmControlTest(unittest.TestCase):
    def test_refuses_to_move_while_disarmed(self):
        c, _, robot, _ = make()
        with self.assertRaises(Refused):
            c.hold('base', 40)
        self.assertEqual(robot.calls, [])

    def test_refuses_to_arm_without_the_arm(self):
        c = ArmControl(lambda: (_ for _ in ()).throw(OSError('no bridge')), clock=Clock())
        c.tick()
        with self.assertRaises(Refused):
            c.set_armed(True)

    def test_hold_moves_then_stops_when_not_renewed(self):
        c, clock, robot, _ = make()
        c.set_armed(True)
        c.hold('shoulder', 30)
        self.assertEqual(robot.calls[-1], ('shoulder', 30))
        clock.t += 0.2
        c.hold('shoulder', 30)             # renewal: no new radio command
        self.assertEqual(robot.calls.count(('shoulder', 30)), 1)
        clock.t += arm_server.HOLD_S + 0.01
        c.tick()
        self.assertEqual(robot.calls[-1], ('shoulder', 0))

    def test_release_stops_that_joint_only(self):
        c, _, robot, _ = make()
        c.set_armed(True)
        c.hold('base', 40)
        c.hold('elbow', -30)
        c.release('base')
        self.assertEqual(robot.calls[-1], ('base', 0))
        self.assertEqual(c.status()['running'], {'base': 0, 'elbow': -30})

    def test_speed_is_clamped(self):
        c, _, robot, _ = make()
        c.set_armed(True)
        c.hold('base', 500)
        self.assertEqual(robot.calls[-1], ('base', arm_server.MAX_SPEED))

    def test_lease_lapse_disarms_and_stops(self):
        c, clock, robot, _ = make()
        c.set_armed(True)
        c.hold('base', 40)
        for _ in range(int(arm_server.LEASE_S / 0.2)):   # keep holding, never renew the lease
            clock.t += 0.2
            c.hold('base', 40)
            c.tick()
        clock.t += 0.3
        c.tick()
        self.assertFalse(c.status()['armed'])
        self.assertIn('stop', robot.calls)
        with self.assertRaises(Refused):
            c.hold('base', 40)

    def test_stop_disarms(self):
        c, _, robot, _ = make()
        c.set_armed(True)
        c.hold('elbow', 30)
        s = c.stop('test')
        self.assertFalse(s['armed'])
        self.assertEqual(robot.calls[-1], 'stop')
        self.assertEqual(s['running'], {})

    def test_lost_arm_disarms(self):
        c, clock, robot, bridge = make()
        c.set_armed(True)
        bridge.ack = False
        clock.t += arm_server.LINK_CHECK_S + 0.1
        c.lease_until = clock.t + 10
        c.tick()
        s = c.status()
        self.assertFalse(s['armed'])
        self.assertFalse(s['link']['arm'])
        self.assertTrue(s['error'])

    def test_radio_failure_while_moving_disarms(self):
        c, _, robot, _ = make()
        c.set_armed(True)
        robot.fail = True
        c.hold('base', 40)
        s = c.status()
        self.assertFalse(s['armed'])
        self.assertIn('no ack', s['error'])


if __name__ == '__main__':
    unittest.main()
