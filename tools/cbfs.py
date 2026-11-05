"""Copy files to/from a CyberBrick core board over its USB REPL.

The stock CyberBrick firmware runs forever from boot.py, which trips up
mpremote, so this interrupts it and talks raw REPL directly.

  cbfs.py PORT ls
  cbfs.py PORT backup DEST_DIR
  cbfs.py PORT put LOCAL REMOTE [LOCAL REMOTE ...]
  cbfs.py PORT exec "python code"
  cbfs.py PORT reset
"""
import binascii
import os
import sys
import time

import serial


class Board:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.interrupt()

    def interrupt(self, timeout=8):
        # retry: the board may still be booting (opening the port can reset it)
        end = time.time() + timeout
        tries = 0
        while True:
            tries += 1
            if tries == 2:
                # no answer: the stock firmware may be asleep; reset the chip
                # through the USB-JTAG bridge (RTS) and catch it while booting
                self.s.dtr = False
                self.s.rts = True
                time.sleep(0.1)
                self.s.rts = False
                time.sleep(0.8)
            for _ in range(3):
                self.s.write(b"\r\x03")
                time.sleep(0.15)
            time.sleep(0.3)
            self.s.reset_input_buffer()
            self.s.write(b"\x01")  # raw REPL
            try:
                self._read_until(b"raw REPL; CTRL-B to exit\r\n>", 1.5)
                return
            except TimeoutError:
                if time.time() > end:
                    raise

    def _read_until(self, marker, timeout):
        buf = b""
        end = time.time() + timeout
        while time.time() < end:
            buf += self.s.read(4096)
            if buf.endswith(marker):
                return buf
        raise TimeoutError(f"waiting for {marker!r}, got {buf[-200:]!r}")

    def exec(self, code, timeout=10):
        self.s.write(code.encode() + b"\x04")
        out = self._read_until(b"\x04>", timeout)
        if not out.startswith(b"OK"):
            raise RuntimeError(f"raw REPL refused code: {out[:200]!r}")
        stdout, _, stderr = out[2:-2].partition(b"\x04")
        if stderr.strip():
            raise RuntimeError(stderr.decode(errors="replace"))
        return stdout.decode(errors="replace")

    def walk(self, path="/"):
        code = (
            "import os\n"
            "def w(p):\n"
            " for n in os.listdir(p):\n"
            "  f=(p.rstrip('/')+'/'+n)\n"
            "  if os.stat(f)[0]&0x4000: w(f)\n"
            "  else: print(f)\n"
            f"w({path!r})"
        )
        return [l for l in self.exec(code).splitlines() if l]

    def get(self, remote):
        out = self.exec(
            "import ubinascii\n"
            f"f=open({remote!r},'rb')\n"
            "while True:\n"
            " b=f.read(512)\n"
            " if not b: break\n"
            " print(ubinascii.b2a_base64(b).decode().strip())\n"
            "f.close()",
            timeout=30,
        )
        return b"".join(binascii.a2b_base64(l) for l in out.splitlines() if l)

    def put(self, local, remote):
        data = open(local, "rb").read()
        parent = remote.rsplit("/", 1)[0]
        if parent:
            self.exec(
                "import os\n"
                f"p=''\nfor d in {parent!r}.strip('/').split('/'):\n"
                " p+='/'+d\n"
                " try: os.mkdir(p)\n"
                " except OSError: pass"
            )
        self.exec(f"import ubinascii\nf=open({remote!r},'wb')")
        for i in range(0, len(data), 256):
            chunk = binascii.b2a_base64(data[i:i + 256]).decode().strip()
            self.exec(f"f.write(ubinascii.a2b_base64({chunk!r}))")
        self.exec("f.close()")

    def reset(self):
        self.s.write(b"\x02")  # back to friendly REPL
        time.sleep(0.1)
        self.s.write(b"import machine; machine.reset()\r")
        time.sleep(0.3)


def main():
    port, cmd, *args = sys.argv[1:]
    b = Board(port)
    if cmd == "ls":
        print("\n".join(b.walk()))
    elif cmd == "backup":
        dest = args[0]
        for f in b.walk():
            path = os.path.join(dest, f.lstrip("/"))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as fh:
                fh.write(b.get(f))
            print("saved", f)
    elif cmd == "put":
        for local, remote in zip(args[::2], args[1::2]):
            b.put(local, remote)
            print("wrote", remote)
    elif cmd == "exec":
        print(b.exec(args[0]), end="")
    elif cmd == "reset":
        b.reset()
        print("reset")


if __name__ == "__main__":
    main()
