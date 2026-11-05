# cyberbrick-bridge

Control CyberBrick robots (L-ONE arm, rover, …) from a Mac, or from a local AI
running on it, through one CyberBrick core board plugged into USB.

```
Mac (Python / AI) --USB serial--> bridge core board --ESP-NOW--> robot's core board + shield
```

The stock CyberBrick radio link can't be driven from a computer, so both boards
run this project's MicroPython firmware instead (`firmware/cbb/cbb_main.py`,
one file for both roles). The stock app files stay on each board, so you can go
back to the CyberBrick app at any time.

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -e .
```

### Bridge board (on the Mac's USB)

```bash
.venv/bin/python tools/deploy.py /dev/cu.usbmodemXXXX bridge
```

The deploy prints the bridge's radio MAC; receivers need it. LED is blue when it's running.

### Robot board

Plug the robot's core board into USB, then:

```bash
.venv/bin/python tools/deploy.py /dev/cu.usbmodemXXXX receiver --name arm --bridge <bridge-mac>
```

The receiver only accepts commands from that bridge. LED colours: amber means waiting,
green means receiving commands, red means the failsafe stopped everything because no
command arrived for 600 ms while something was moving.

### Robot board without USB (over the air)

Stock receivers run Python "code" effects that the CyberBrick app sends them.
`tools/make_ota_config.py` builds an app config whose code effect installs a
small loader (`firmware/cbb/loader.py`), so the robot never needs a cable:

1. Put stock firmware back on the bridge board so it works as a normal remote:
   `tools/deploy.py PORT restore`.
2. `.venv/bin/python tools/make_ota_config.py --name arm --bridge <bridge-mac> -o ota/arm-installer.json`
3. In the CyberBrick app, choose Create Configuration > Import from File and pick that JSON.
   Then connect and bind the remote and the robot, and Send Configuration.
4. Long-press any remote button (K1–K4). The robot writes the loader and reboots.
5. Flash the bridge again (`tools/deploy.py PORT bridge`). Within 2 minutes of the
   robot booting, run `cbb scan` and then `cbb update <mac>` to push the full firmware.

If the loader hears nothing from the bridge for 2 minutes, it starts the stock
firmware for that boot. The cbb firmware also falls back to stock if it crashes. Either way
the robot stays reachable from the app. `cbb update <mac>` pushes new firmware over the
radio later, and `cbb restore-stock <mac>` goes back to stock.

### Back to stock CyberBrick firmware

```bash
.venv/bin/python tools/deploy.py /dev/cu.usbmodemXXXX restore
```

Full pre-install copies of every board are in `backups/<board-mac>/`.

## Use

```bash
.venv/bin/cbb ping
.venv/bin/cbb scan                       # robots paired to this bridge
.venv/bin/cbb jog <mac> motor 1 40 -t 0.5
.venv/bin/cbb jog <mac> speed 1 30 -t 0.5
.venv/bin/cbb move <mac> shoulder -30 -t 1
.venv/bin/cbb stop <mac>
```

```python
from cyberbrick_bridge import Bridge, Robot

with Bridge() as b, Robot(b, "<arm mac>") as arm:
    arm.move("base", 40, duration=0.8)    # gearmotor, -100..100
    arm.move_for("shoulder", -30, 1.0)    # 360° servo, blocking
    # no gripper yet: add "gripper": ("angle", 3) to ARM_JOINTS once it's fitted
```

Opening the port soft-resets the bridge (about 2.5 s), so keep one `Bridge` open
for a long-running agent rather than reconnecting for every command.

Joint-to-port mapping is `ARM_JOINTS` in `cyberbrick_bridge/robot.py`. Use
`cbb jog` to check which port drives which joint. For a different robot, pass your
own `joints={...}` to `Robot`.

## Web app

```bash
.venv/bin/python web/arm_server.py
```

Open http://localhost:8020/ on the Mac, or http://robo-arm.local:8020/ from a phone on the home
Wi-Fi. Other devices need the password in `web/.dashboard-password`, which is created on first
run and gitignored. STOP works without logging in.

Arm, then hold a joint's buttons or keys: base <kbd>A</kbd>/<kbd>D</kbd>, shoulder
<kbd>W</kbd>/<kbd>S</kbd>, elbow <kbd>↑</kbd>/<kbd>↓</kbd>. Letting go stops the joint.
<kbd>Esc</kbd> or <kbd>Space</kbd> is STOP. Each joint has a speed slider and a "Flip direction"
box, both remembered in the browser.

Safety layers: the page's 3 s arm lease, a hold that lapses 0.35 s after the page stops renewing
it, and the arm's own 0.6 s radio failsafe. The server owns the bridge's USB port, so the rover's
drive server can't run at the same time. Tests: `.venv/bin/python -m unittest web/test_arm_server.py`.

## Protocol

Mac → bridge: one JSON object per line, for example
`{"id": 1, "op": "send", "to": "<mac>|*", "msg": {...}}`.
Bridge → Mac: lines that start with `@` followed by JSON. Replies carry the request's
`id`, and robot messages arrive as `{"ev": "rx", "from": mac, "msg": {...}}`.

Robot messages: `motor` / `servo_speed` / `servo_angle` (`ch`, `v`), `stop`,
`led` (`rgb`), `ping` / `hello`, `ka` (keepalive), `batch` (`cmds`).
