# cyberbrick-bridge boot.
#
# If the cbb firmware fails to start or crashes, fall back to the stock
# CyberBrick firmware (saved as /boot_stock.py on install) so the board can
# still be reached from the CyberBrick app without a USB cable.
import sys

sys.path.append("/cbb")
try:
    import cbb_main

    cbb_main.main()
except Exception as ex:
    print("[cbb] firmware crashed, starting stock CyberBrick firmware:", ex)
    exec(open("/boot_stock.py").read())
