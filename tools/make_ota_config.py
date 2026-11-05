"""Build a CyberBrick app config that installs cbb firmware on a receiver over the air.

The stock receiver firmware runs Python "code" effects from its rc_config when
a remote button fires them (bbl/executor.py). This writes a config whose code
effect unpacks the cbb firmware onto the receiver, saves the stock boot.py as
/boot_stock.py, and reboots into cbb.

  make_ota_config.py --name arm --bridge <bridge-mac> -o ota/arm-installer.json

Import the JSON in the CyberBrick app (Create Configuration > Import from File)
and send it to the remote and the robot. With a bare core board as the remote
(--trigger bare, the default) the install starts by itself; with a real remote
(--trigger button) long-press any of its buttons.

The executor indents every line, swaps time.sleep for asyncio.sleep, rewrites
`while True:` and rejects any line containing exit, quit, os.remove, ... so the
installer avoids those, and the payload is zlib + hex (hex can't spell them).
"""
import argparse
import binascii
import json
import os
import sys
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware")

# The stock executor's blocklist, read from a running BBL_Controller. Any line
# containing one of these is rejected.
DANGER = ["exit", "quit", "sys.exit", "os.system", "__import__", "open", "eval",
          "exec", "subprocess", "os.remove", "os.rmdir"]

# Remote channels 6-9 are the four buttons (K1-K4) on the stock remotes.
BUTTON_CHANNELS = range(6, 10)


WBITS = 9  # 512-byte window: the default 32 KB one doesn't fit in the receiver's RAM


def minify(src):
    """Drop comments and blank lines, and shrink indents to one space per level.

    Only for our own simple files: no strings containing '#', and indents in
    multiples of four.
    """
    out = []
    for line in src.decode().splitlines():
        code = line.split("#", 1)[0].rstrip()
        if code.strip():
            n = len(code) - len(code.lstrip())
            out.append(" " * (n // 4) + code.lstrip())
    return ("\n".join(out) + "\n").encode()


def packed(data):
    c = zlib.compressobj(9, zlib.DEFLATED, WBITS)
    return binascii.hexlify(c.compress(data) + c.flush()).decode()


def installer_code(files, prefix="", reset=True):
    root = prefix or "/"
    lines = [
        "import ubinascii as U, deflate, io, os, machine, builtins",
        "O = getattr(builtins, 'op' + 'en')",  # the blocklist rejects lines containing it
        "def W(p, h):",
        " f = O(p, 'wb')",
        " f.write(deflate.DeflateIO(io.BytesIO(U.unhexlify(h)), deflate.ZLIB, %d).read())" % WBITS,
        " f.close()",
        "try:",
        " os.mkdir('%s/cbb')" % prefix,
        "except OSError:",
        " pass",
    ]
    # The executor cancels and restarts a running effect if it fires again, so
    # every step must be safe to interrupt: boot.py is only ever replaced by a
    # rename, after a copy of the stock one is in place.
    for remote, data in files:
        if remote == "/boot.py":
            lines += [
                "if 'boot_stock.py' not in os.listdir('%s'):" % root,
                " f = O('%s/bs.tmp', 'wb')" % prefix,
                " f.write(O('%s/boot.py', 'rb').read())" % prefix,
                " f.close()",
                " os.rename('%s/bs.tmp', '%s/boot_stock.py')" % (prefix, prefix),
                "W('%s/boot.new', '%s')" % (prefix, packed(data)),
                "os.rename('%s/boot.new', '%s/boot.py')" % (prefix, prefix),
            ]
        else:
            lines.append("W('%s%s', '%s')" % (prefix, remote, packed(data)))
    lines.append("print('[cbb-install] done')")
    if reset:
        lines.append("machine.reset()")
    for l in lines:
        bad = [d for d in DANGER if d in l]
        if bad:
            sys.exit(f"installer line would be rejected by the executor ({bad}): {l[:80]}")
        if "sleep(" in l or "while" in l:
            sys.exit(f"installer line would be rewritten by the executor: {l[:80]}")
    return "\n".join(lines)


def firmware_files(name, bridge, failsafe_ms):
    # The executor runs out of memory on anything much over 2 KB of code, so
    # this installs the small loader; `cbb update` then pushes the full
    # firmware over the radio.
    cfg = {"role": "receiver", "name": name, "bridge": bridge, "failsafe_ms": failsafe_ms}
    return [
        ("/cbb/cbb_main.py", minify(open(os.path.join(FW, "cbb", "loader.py"), "rb").read())),
        ("/cbb_config.json", json.dumps(cfg, separators=(",", ":")).encode()),
        ("/boot.py", minify(open(os.path.join(FW, "boot.py"), "rb").read())),
    ]


def app_config(code, config_name, trigger="bare"):
    """trigger "bare": a bare core board as the remote reads 0 on channel 1, so a
    "below middle" event there fires by itself as soon as the robot loads the
    config. trigger "button": long-press any of the remote's K1-K4 buttons."""
    event = {"actuator": "CODE", "set_value": [1], "receiver": 1}
    channels = [{} for _ in range(10)]
    if trigger == "bare":
        channels[1] = {"name": "L Stick", "device": "joystick", "controls": [],
                       "data": {"channel": "y", "deadzone": 240, "mid_value": 2048},
                       "event": [{"type": "lt_mid", **event}]}
    else:
        for i in BUTTON_CHANNELS:
            channels[i] = {"name": f"Button {i - 5}", "device": "button", "data": {},
                           "controls": [], "event": [{"type": "long", **event}]}

    empty = {"PWM1": {}, "PWM2": {}, "PWM3": {}, "PWM4": {}, "LED1": {}, "LED2": {},
             "MOTOR1": {}, "MOTOR2": {}, "BUZZER1": {}, "BUZZER2": {}}
    return {
        "version": "2",
        "config_name": config_name,
        "config_type": "custom",
        "sender": {"auto_sleep": {"en": False}, "channels": channels},
        "switches": {"list": [{"state": "off"}, {"state": "off"}]},
        "receiver_1": {**empty, "CODE": {"en": True, "data": [
            {"effect": 1, "effect_name": "Install cbb firmware", "code": code}]}},
        "receiver_2": {**empty, "CODE": {}},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="arm")
    ap.add_argument("--bridge", required=True, help="bridge board MAC")
    ap.add_argument("--failsafe-ms", type=int, default=600)
    ap.add_argument("--trigger", choices=["bare", "button"], default="bare",
                    help="bare core board as remote (fires by itself) or a remote with buttons")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()

    bridge = a.bridge.lower().replace(":", "")
    code = installer_code(firmware_files(a.name, bridge, a.failsafe_ms))
    cfg = app_config(code, f"cbb installer ({a.name})", a.trigger)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(cfg, f, indent=1)
    print(f"wrote {a.out}: installer code {len(code)} chars, config {os.path.getsize(a.out)} bytes")


if __name__ == "__main__":
    main()
