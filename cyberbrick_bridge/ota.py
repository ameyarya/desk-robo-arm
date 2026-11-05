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


def _request(bridge, mac, msg, retries=6):
    for attempt in range(retries):
        bridge.drain()
        if bridge.send(mac, msg):
            try:
                r = _wait_reply(bridge, mac, msg["op"])
                if r.get("ok"):
                    return r
                raise RuntimeError(f"{msg['op']} failed: {r}")
            except TimeoutError:
                pass
        time.sleep(0.1 * (attempt + 1))
    raise TimeoutError(f"robot {mac} did not confirm {msg['op']}")


def push_file(bridge, mac, data, remote, reset=False, progress=print):
    """Write `data` (bytes) to `remote` on the robot, verified by crc32."""
    mac = mac.lower().replace(":", "")
    for off in range(0, len(data), CHUNK):
        chunk = data[off:off + CHUNK]
        _request(bridge, mac, {"op": "fput", "p": remote, "o": off,
                               "h": binascii.hexlify(chunk).decode()})
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
