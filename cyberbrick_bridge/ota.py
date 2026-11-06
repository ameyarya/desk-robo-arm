"""Update files on a robot's board over the radio (no USB needed)."""
import binascii
import queue
import time
import zlib

CHUNK = 90  # bytes per packet; hex-encoded it keeps the JSON under ESP-NOW's 250


def _wait_reply(bridge, mac, op, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            ev = bridge.events.get(timeout=max(0.0, end - time.time()))
        except queue.Empty:
            break
        if ev.get("ev") == "rx" and ev.get("from") == mac and ev["msg"].get("op") == op:
            return ev["msg"]
    raise TimeoutError(f"robot {mac} did not answer {op}")


def _request(bridge, mac, msg, give_up_s=15.0):
    """Send until the robot confirms. The radio has dead spells of a few seconds now and then,
    so this keeps trying (with a growing pause, at most 0.5 s) for up to `give_up_s`."""
    end = time.time() + give_up_s
    attempt = 0
    while True:
        bridge.drain()
        if bridge.send(mac, msg):
            try:
                r = _wait_reply(bridge, mac, msg["op"], timeout=1.0)
                if r.get("ok"):
                    return r
                raise RuntimeError(f"{msg['op']} failed: {r}")
            except TimeoutError:
                pass
        attempt += 1
        if time.time() > end:
            raise TimeoutError(f"robot {mac} did not confirm {msg['op']} in {give_up_s:.0f} s "
                               f"({attempt} tries): is it switched on and in range?")
        time.sleep(min(0.5, 0.05 * attempt))


def push_file(bridge, mac, data, remote, reset=False, progress=print, pause=0.0):
    """Write `data` (bytes) to `remote` on the robot, verified by crc32.

    If it fails partway, the staged copy (remote + ".part") is shrunk to 1 byte so it doesn't
    eat the board's small flash (a full filesystem makes every later write stall the radio).
    """
    mac = mac.lower().replace(":", "")
    try:
        _push(bridge, mac, data, remote, reset, progress, pause)
    except Exception:
        try:
            _request(bridge, mac, {"op": "fput", "p": remote, "o": 0, "h": "00"}, give_up_s=3.0)
        except Exception:
            pass
        raise


def _push(bridge, mac, data, remote, reset, progress, pause):
    for off in range(0, len(data), CHUNK):
        chunk = data[off:off + CHUNK]
        _request(bridge, mac, {"op": "fput", "p": remote, "o": off,
                               "h": binascii.hexlify(chunk).decode()})
        if pause:
            time.sleep(pause)   # older firmware writes each chunk to flash: let it finish
        if progress and (off // CHUNK) % 20 == 0:
            progress(f"{remote}: {off + len(chunk)}/{len(data)} bytes")
    crc = zlib.crc32(data) & 0xFFFFFFFF
    _request(bridge, mac, {"op": "fdone", "p": remote, "n": len(data),
                           "crc": crc, "reset": reset})
    if progress:
        progress(f"{remote}: done ({len(data)} bytes, crc {crc:08x})")


def reset(bridge, mac):
    bridge.send(mac.lower().replace(":", ""), {"op": "reset"})


def restore_stock(bridge, mac):
    """Put the stock CyberBrick boot.py back on the robot and reboot it."""
    _request(bridge, mac.lower().replace(":", ""), {"op": "restore_stock"})
