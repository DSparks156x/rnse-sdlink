"""sdlink.py -- tiny pyserial helper for the sdemu firmware console (Windows COMx or Pi /dev/ttyACM0).

    from sdlink import Link
    with Link() as t:            # auto-detects the Teensy (USB VID 0x16C0)
        print(t.cmd("ver"))       # -> list of reply lines
"""
import sys, time

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    sys.exit("pyserial missing:  <python> -m pip install pyserial")

TEENSY_VID = 0x16C0


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == TEENSY_VID:
            return p.device
    for p in ports:  # Pi / Linux fallback
        if "ttyACM" in p.device:
            return p.device
    raise SystemExit("no Teensy serial port found (is it plugged in and running sdemu?). Ports: "
                     + ", ".join(p.device for p in ports))


class Link:
    def __init__(self, port=None, timeout=0.2):
        self.port = port or find_port()
        self.ser = serial.Serial(self.port, 115200, timeout=timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self._buf = b""

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def write_line(self, s):
        self.ser.write((s + "\n").encode())

    def read_line(self, timeout=1.0):
        """one decoded line without the newline, or None on timeout"""
        end = time.time() + timeout
        while True:
            i = self._buf.find(b"\n")
            if i >= 0:
                line, self._buf = self._buf[:i], self._buf[i + 1:]
                return line.decode(errors="replace").rstrip("\r")
            if time.time() > end:
                return None
            chunk = self.ser.read(max(1, self.ser.in_waiting))
            if chunk:
                self._buf += chunk

    def read_exact(self, n, timeout=10.0):
        """raw bytes (used by the USB throughput test)"""
        out = bytearray(self._buf[:n])
        self._buf = self._buf[n:]
        end = time.time() + timeout
        while len(out) < n and time.time() < end:
            chunk = self.ser.read(min(n - len(out), 1 << 20))
            if chunk:
                out += chunk
                end = time.time() + timeout
        return bytes(out)

    def cmd(self, s, until=None, timeout=2.0, quiet_prefixes=("STAT", "C ", "R ", "D ", "E ")):
        """send a command; collect reply lines until a line starts with `until` (or timeout).
        Background log lines (STAT/C/R/D/E) are skipped unless `until` asks for them."""
        self.write_line(s)
        lines, end = [], time.time() + timeout
        while time.time() < end:
            ln = self.read_line(timeout=max(0.01, end - time.time()))
            if ln is None:
                break
            if until is None and ln.startswith(quiet_prefixes):
                continue
            lines.append(ln)
            if until and ln.startswith(until):
                break
            if until is None:
                end = min(end, time.time() + 0.15)  # single-reply commands: stop shortly after the reply
        return lines


def parse_kv(line):
    """'STAT mode=card vdd=1 3.28V clk_avg=...' -> dict of the key=value tokens"""
    d = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                d[k] = int(v)
            except ValueError:
                try:
                    d[k] = float(v.rstrip("V%"))
                except ValueError:
                    d[k] = v
    return d


def get_counters(link):
    """returns merged dict of all CTR lines"""
    link.write_line("ctr")
    d, got, end = {}, 0, time.time() + 2
    while got < 4 and time.time() < end:
        ln = link.read_line(0.5)
        if ln and ln.startswith("CTR"):
            d.update(parse_kv(ln))
            got += 1
    return d


def get_status(link):
    link.write_line("status")
    end = time.time() + 2
    while time.time() < end:
        ln = link.read_line(0.5)
        if ln and ln.startswith("STAT"):
            return parse_kv(ln), ln
    return {}, ""
