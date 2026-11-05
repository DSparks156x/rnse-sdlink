#!/usr/bin/env python3
"""Virtual touchscreen for the Pi, no packages. Web page shows the screen; click + drag there become real touches.

    sudo python3 vtouch.py serve [--port 8088]     then open http://<pi-address>:8088 on the Hudiy Wi-Fi
    sudo python3 vtouch.py swipe SECS              scripted map drags, for repeatable motion tests

Bench tool only: the page has no login, and anyone on the Pi's Wi-Fi can use it while it runs. Don't leave it running in the car.
"""
import fcntl, http.server, json, os, struct, subprocess, sys, threading, time, urllib.parse

W, H = 800, 480
EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
BTN_TOUCH = 0x14A
ABS_MT_SLOT, ABS_MT_POSITION_X, ABS_MT_POSITION_Y, ABS_MT_TRACKING_ID = 0x2F, 0x35, 0x36, 0x39
ABS_X, ABS_Y = 0, 1
UI_SET_EVBIT, UI_SET_KEYBIT, UI_SET_ABSBIT, UI_SET_PROPBIT = 0x40045564, 0x40045565, 0x40045567, 0x4004556E
UI_DEV_SETUP, UI_ABS_SETUP, UI_DEV_CREATE, UI_DEV_DESTROY = 0x405C5503, 0x401C5504, 0x5501, 0x5502
INPUT_PROP_DIRECT = 1


class Touch:
    def __init__(self):
        self.fd = os.open('/dev/uinput', os.O_WRONLY | os.O_NONBLOCK)
        self.tid = 100
        self.lock = threading.Lock()
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
        fcntl.ioctl(self.fd, UI_SET_KEYBIT, BTN_TOUCH)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
        fcntl.ioctl(self.fd, UI_SET_PROPBIT, INPUT_PROP_DIRECT)
        for code, mx in ((ABS_X, W - 1), (ABS_Y, H - 1), (ABS_MT_SLOT, 0), (ABS_MT_POSITION_X, W - 1), (ABS_MT_POSITION_Y, H - 1), (ABS_MT_TRACKING_ID, 65535)):
            fcntl.ioctl(self.fd, UI_SET_ABSBIT, code)
            fcntl.ioctl(self.fd, UI_ABS_SETUP, struct.pack('<HHiiiiii', code, 0, 0, 0, mx, 0, 0, 0))
        name = b'vtouch'.ljust(80, b'\0')
        fcntl.ioctl(self.fd, UI_DEV_SETUP, struct.pack('<HHHH', 3, 0x1234, 0x5678, 1) + name + struct.pack('<I', 0))
        fcntl.ioctl(self.fd, UI_DEV_CREATE)
        time.sleep(0.5)

    def _ev(self, t, c, v):
        os.write(self.fd, struct.pack('<qqHHi', 0, 0, t, c, v))

    def _pos(self, x, y):
        x, y = max(0, min(W - 1, int(x))), max(0, min(H - 1, int(y)))
        self._ev(EV_ABS, ABS_MT_POSITION_X, x); self._ev(EV_ABS, ABS_MT_POSITION_Y, y)
        self._ev(EV_ABS, ABS_X, x); self._ev(EV_ABS, ABS_Y, y)

    def down(self, x, y):
        with self.lock:
            self.tid = (self.tid + 1) & 0x7FFF
            self._ev(EV_ABS, ABS_MT_SLOT, 0); self._ev(EV_ABS, ABS_MT_TRACKING_ID, self.tid)
            self._pos(x, y); self._ev(EV_KEY, BTN_TOUCH, 1); self._ev(EV_SYN, 0, 0)

    def move(self, x, y):
        with self.lock:
            self._ev(EV_ABS, ABS_MT_SLOT, 0); self._pos(x, y); self._ev(EV_SYN, 0, 0)

    def up(self):
        with self.lock:
            self._ev(EV_ABS, ABS_MT_SLOT, 0); self._ev(EV_ABS, ABS_MT_TRACKING_ID, -1)
            self._ev(EV_KEY, BTN_TOUCH, 0); self._ev(EV_SYN, 0, 0)


PAGE = b"""<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>RNS-E touch</title>
<style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif;text-align:center}#i{width:100%;max-width:800px;touch-action:none;user-select:none;-webkit-user-drag:none;cursor:crosshair;display:block;margin:auto}</style>
<img id=i draggable=false src=/shot><div>click or drag on the picture = touch on the Pi (the picture here refreshes slowly; watch the RNS-E)</div>
<script>
const i=document.getElementById('i');let dn=0,q=null,busy=0;
function xy(e){const r=i.getBoundingClientRect();return [Math.round((e.clientX-r.left)*800/r.width),Math.round((e.clientY-r.top)*480/r.height)]}
function send(t,p){fetch('/t?e='+t+'&x='+p[0]+'&y='+p[1],{method:'POST'})}
i.onpointerdown=e=>{e.preventDefault();i.setPointerCapture(e.pointerId);dn=1;send('d',xy(e))};
i.onpointermove=e=>{if(dn){q=xy(e);if(!busy){busy=1;setTimeout(()=>{if(q&&dn)send('m',q);busy=0},16)}}};
i.onpointerup=i.onpointercancel=e=>{dn=0;send('u',xy(e))};
setInterval(()=>{const n=new Image();n.onload=()=>{i.src=n.src};n.src='/shot?'+Date.now()},700);
</script>"""


def shot():
    return subprocess.run(['sudo', '-u', 'pi', 'env', 'XDG_RUNTIME_DIR=/run/user/1000', 'WAYLAND_DISPLAY=wayland-0', 'grim', '-t', 'jpeg', '-q', '70', '-'],
                          capture_output=True, timeout=3).stdout


def serve(port):
    t = Touch()

    class H(http.server.BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        def log_message(self, *a):
            pass
        def _send(self, body, ctype='text/plain'):
            self.send_response(200); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            if self.path.startswith('/shot'):
                self._send(shot(), 'image/jpeg')
            else:
                self._send(PAGE, 'text/html')
        def do_POST(self):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            e, x, y = q.get('e', ['?'])[0], int(q.get('x', ['0'])[0]), int(q.get('y', ['0'])[0])
            if e == 'd': t.down(x, y)
            elif e == 'm': t.move(x, y)
            elif e == 'u': t.up()
            self._send(b'ok')

    print('vtouch on port', port, flush=True)
    http.server.ThreadingHTTPServer(('0.0.0.0', port), H).serve_forever()


def swipe(secs):
    t = Touch()
    end = time.time() + secs
    n = 0
    while time.time() < end:
        x0, y0, x1, y1 = [(600, 200, 200, 200), (200, 200, 600, 200), (450, 80, 450, 300), (450, 300, 450, 80)][n % 4]
        t.down(x0, y0)
        for k in range(1, 31):
            t.move(x0 + (x1 - x0) * k / 30, y0 + (y1 - y0) * k / 30)
            time.sleep(1 / 60)
        t.up()
        n += 1
        time.sleep(0.15)
    print('swipes', n)


if __name__ == '__main__':
    a = sys.argv[1:]
    if a[:1] == ['serve']:
        serve(int(a[a.index('--port') + 1]) if '--port' in a else 8088)
    elif a[:1] == ['swipe']:
        swipe(float(a[1]))
    else:
        print(__doc__)
