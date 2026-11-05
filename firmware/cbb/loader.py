# Minimal cbb receiver, small enough for the stock CyberBrick code executor to
# install over the air (it has ~20 KB to work with). It only accepts file
# writes from the bridge; `cbb update MAC` pushes the full firmware, which
# replaces this file.
#
# Ops (from the bridge only): fput {p, o, h}, fdone {p, n, crc, reset},
# reset, restore_stock; anything else just gets a reply (used by scan).
# If nothing arrives from the bridge for wait_ms after boot, it raises so
# boot.py starts the stock firmware for this boot.
import machine, network, espnow, ujson, ubinascii, os, time

def main(cfg='/cbb_config.json', wait_ms=120000):
    c = ujson.load(open(cfg))
    t = time.ticks_ms()
    w = network.WLAN(network.STA_IF)
    w.active(True)
    w.config(channel=1)
    e = espnow.ESPNow()
    e.active(True)
    b = ubinascii.unhexlify(c['bridge'])
    e.add_peer(b)
    heard = 0
    while 1:
        m, msg = e.recv(1000)
        if m != b:
            if not heard and time.ticks_diff(time.ticks_ms(), t) > wait_ms:
                e.active(False)
                raise OSError('no bridge')
            continue
        heard = 1
        op = '?'
        try:
            q = ujson.loads(msg)
            op = q['op']
            r = {'ok': 1, 'name': c['name'], 'version': 'loader'}
            p = q.get('p', '') + '.part'
            if op == 'fput':
                h = ubinascii.unhexlify(q['h'])
                o = q['o']
                s = os.stat(p)[6] if o else 0
                if s == o:
                    f = open(p, 'ab' if o else 'wb')
                    f.write(h)
                    f.close()
                elif s != o + len(h):  # equal: a retried chunk already written
                    raise OSError('offset')
                r['o'] = o
            elif op == 'fdone':
                d = open(p, 'rb').read()
                if len(d) != q['n'] or ubinascii.crc32(d) & 0xFFFFFFFF != q['crc']:
                    raise OSError('check')
                os.rename(p, q['p'])
            elif op == 'restore_stock':
                os.rename('/boot_stock.py', '/boot.py')
        except Exception as x:
            r = {'ok': 0, 'err': str(x)}
        r['op'] = op
        e.send(b, ujson.dumps(r))
        if r['ok'] and (op in ('reset', 'restore_stock') or (op == 'fdone' and q.get('reset'))):
            time.sleep_ms(200)
            machine.reset()
