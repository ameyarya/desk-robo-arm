"""Install or remove cyberbrick-bridge firmware on a CyberBrick core board.

  deploy.py PORT bridge
  deploy.py PORT receiver --name arm --bridge <bridge-mac>
  deploy.py PORT restore          # put the stock CyberBrick boot.py back

Before the first install on a board, every file on it is copied to
backups/<board-mac>/. The stock CyberBrick app files are left on the board,
so restoring only swaps boot.py back.
"""
import argparse
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
from cbfs import Board  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW = os.path.join(ROOT, "firmware")


def board_mac(b):
    # unique_id() is the ESP32's base MAC, which is also the Wi-Fi STA MAC
    # ESP-NOW uses; reading it doesn't need the radio up.
    out = b.exec(
        "import machine\nprint(''.join('%02x'%x for x in machine.unique_id()))"
    )
    return out.strip()


def backup(b, mac):
    dest = os.path.join(ROOT, "backups", mac)
    if os.path.exists(os.path.join(dest, "boot.py")):
        print(f"backup already exists: backups/{mac}")
        return dest
    for f in b.walk():
        path = os.path.join(dest, f.lstrip("/"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            fh.write(b.get(f))
    print(f"backed up board to backups/{mac}")
    return dest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("role", choices=["bridge", "receiver", "restore"])
    ap.add_argument("--name", default="robot")
    ap.add_argument("--bridge", help="bridge board MAC (receiver only)")
    ap.add_argument("--failsafe-ms", type=int, default=600)
    a = ap.parse_args()

    b = Board(a.port)
    mac = board_mac(b)
    print("board", mac)
    files = b.walk()

    if a.role == "restore":
        stock = os.path.join(ROOT, "backups", mac, "boot.py")
        if not os.path.exists(stock):
            sys.exit(f"no backup for {mac}; nothing to restore from")
        b.put(stock, "/boot.py")
        b.exec("import os\ntry: os.remove('/cbb_config.json')\nexcept OSError: pass")
        b.reset()
        print("stock CyberBrick boot.py restored; board reset")
        return

    if a.role == "receiver":
        if not a.bridge:
            sys.exit("--bridge MAC is required for a receiver")
        missing = [f for f in ("/bbl/servos.py", "/bbl/motors.py") if f not in files]
        if missing:
            sys.exit(f"board is missing CyberBrick drivers: {missing}")

    if "/cbb/cbb_main.py" not in files:
        backup(b, mac)
    if "/boot_stock.py" not in files:
        # boot.py's crash fallback runs this; the stock app files stay on the board
        b.put(os.path.join(ROOT, "backups", mac, "boot.py"), "/boot_stock.py")

    cfg = {"role": a.role}
    if a.role == "receiver":
        cfg.update(name=a.name, bridge=a.bridge.lower().replace(":", ""),
                   failsafe_ms=a.failsafe_ms)
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as t:
        json.dump(cfg, t)

    b.put(os.path.join(FW, "cbb", "cbb_main.py"), "/cbb/cbb_main.py")
    b.put(t.name, "/cbb_config.json")
    b.put(os.path.join(FW, "boot.py"), "/boot.py")
    os.unlink(t.name)
    print("installed", json.dumps(cfg))
    b.reset()
    print("board reset")


if __name__ == "__main__":
    main()
