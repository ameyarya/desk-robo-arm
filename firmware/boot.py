# cyberbrick-bridge boot.
#
# If the cbb firmware fails to start or crashes, save the traceback to
# /cbb_crash.txt (the next cbb run reports it to the Mac in its hello reply)
# and fall back to the stock CyberBrick firmware (saved as /boot_stock.py on
# install) so the board can still be reached from the CyberBrick app.
import sys

sys.path.append("/cbb")
try:
    import cbb_main

    cbb_main.main()
except Exception as ex:
    print("[cbb] firmware crashed, starting stock CyberBrick firmware:", ex)
    try:
        import io

        buf = io.StringIO()
        sys.print_exception(ex, buf)
        with open("/cbb_crash.txt", "w") as f:
            f.write(buf.getvalue())
    except Exception:
        pass
    exec(open("/boot_stock.py").read())
