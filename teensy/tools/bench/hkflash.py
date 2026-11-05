"""HalfKay (Teensy 4.1 bootloader) HID flasher with patient timeouts. usage: hkflash.py firmware.hex
Same block protocol as teensy_loader_cli (1024 B data + 64 B header, first block triggers the flash erase) but the
first block may take as long as it needs. teensy_loader_cli gives up after 5 s and its retry starts over.
Windows only, ctypes (nothing to install). The Teensy must already be in the bootloader (teensy_reboot -s)."""
import ctypes as ct
import sys
import threading
import time
from ctypes import wintypes as wt

BLOCK = 1024
FLASH_BASE = 0x60000000
FLASH_SIZE = 8126464
FIRST_TIMEOUT = 180.0
NEXT_TIMEOUT = 5.0
NEXT_RETRIES = 5

setupapi = ct.WinDLL("setupapi", use_last_error=True)
hid = ct.WinDLL("hid", use_last_error=True)
k32 = ct.WinDLL("kernel32", use_last_error=True)


class GUID(ct.Structure):
    _fields_ = [("a", wt.DWORD), ("b", wt.WORD), ("c", wt.WORD), ("d", ct.c_ubyte * 8)]


class IFACE(ct.Structure):
    _fields_ = [("cbSize", wt.DWORD), ("guid", GUID), ("flags", wt.DWORD), ("res", ct.c_size_t)]


setupapi.SetupDiGetClassDevsW.restype = wt.HANDLE
setupapi.SetupDiGetClassDevsW.argtypes = [ct.POINTER(GUID), wt.LPCWSTR, wt.HWND, wt.DWORD]
setupapi.SetupDiEnumDeviceInterfaces.argtypes = [wt.HANDLE, ct.c_void_p, ct.POINTER(GUID), wt.DWORD, ct.POINTER(IFACE)]
setupapi.SetupDiGetDeviceInterfaceDetailW.argtypes = [wt.HANDLE, ct.POINTER(IFACE), ct.c_void_p, wt.DWORD,
                                                      ct.POINTER(wt.DWORD), ct.c_void_p]
setupapi.SetupDiDestroyDeviceInfoList.argtypes = [wt.HANDLE]
k32.CreateFileW.restype = wt.HANDLE
k32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ct.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE]
k32.WriteFile.argtypes = [wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p]
k32.CloseHandle.argtypes = [wt.HANDLE]
INVALID = wt.HANDLE(-1).value


def find_halfkay():
    g = GUID()
    hid.HidD_GetHidGuid(ct.byref(g))
    h = setupapi.SetupDiGetClassDevsW(ct.byref(g), None, None, 0x12)   # DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    try:
        i = 0
        while True:
            d = IFACE(); d.cbSize = ct.sizeof(IFACE)
            if not setupapi.SetupDiEnumDeviceInterfaces(h, None, ct.byref(g), i, ct.byref(d)):
                return None
            i += 1
            need = wt.DWORD()
            setupapi.SetupDiGetDeviceInterfaceDetailW(h, ct.byref(d), None, 0, ct.byref(need), None)
            buf = ct.create_string_buffer(need.value)
            ct.cast(buf, ct.POINTER(wt.DWORD))[0] = 8                       # cbSize on 64-bit
            if not setupapi.SetupDiGetDeviceInterfaceDetailW(h, ct.byref(d), buf, need, None, None):
                continue
            path = ct.wstring_at(ct.addressof(buf) + 4)
            if "vid_16c0&pid_0478" in path.lower():
                return path
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(h)


res_err = {}


def write_report(dev, data, timeout):
    """one blocking WriteFile in a thread, so a hung write can't hang us. returns seconds taken, or None"""
    buf = ct.create_string_buffer(b"\0" + data)        # report id 0
    res = {}

    def run():
        n = wt.DWORD()
        res["ok"] = k32.WriteFile(dev, buf, len(data) + 1, ct.byref(n), None)
        res["err"] = ct.get_last_error()

    t0 = time.time()
    th = threading.Thread(target=run, daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        return None
    if not res.get("ok"):
        res_err[res.get("err")] = res_err.get(res.get("err"), 0) + 1
        return None
    return time.time() - t0


def load_hex(path):
    img, base = {}, 0
    for line in open(path):
        line = line.strip()
        if not line.startswith(":"):
            continue
        b = bytes.fromhex(line[1:])
        n, addr, typ = b[0], (b[1] << 8) | b[2], b[3]
        if typ == 0:
            for k in range(n):
                a = base + addr + k
                if a >= FLASH_BASE:
                    a -= FLASH_BASE
                if a >= FLASH_SIZE:
                    sys.exit(f"hex address out of range: {a:#x}")
                img[a] = b[4 + k]
        elif typ == 2:
            base = ((b[4] << 8) | b[5]) << 4
        elif typ == 4:
            base = ((b[4] << 8) | b[5]) << 16
    return img


def main():
    img = load_hex(sys.argv[1])
    top = max(img) + 1
    print(f"{len(img)} bytes, top {top:#x}")
    path = None
    for _ in range(50):
        path = find_halfkay()
        if path:
            break
        time.sleep(0.2)
    if not path:
        sys.exit("no HalfKay device (run teensy_reboot -s first)")
    dev = k32.CreateFileW(path, 0xC0000000, 3, None, 3, 0, None)      # R/W, share R/W, OPEN_EXISTING
    if dev == INVALID:
        sys.exit(f"cannot open HalfKay: {ct.get_last_error()}")
    first = True
    sent = 0
    t_all = time.time()
    for addr in range(0, top, BLOCK):
        data = bytes(img.get(addr + i, 0xFF) for i in range(BLOCK))
        if not first and data == b"\xFF" * BLOCK:
            continue
        pkt = bytes([addr & 255, (addr >> 8) & 255, (addr >> 16) & 255]) + bytes(61) + data
        t_blk = time.time()
        tries = 0
        while True:                  # HalfKay stalls (error 31) while not ready: keep offering the block
            tries += 1
            dt = write_report(dev, pkt, FIRST_TIMEOUT if first else NEXT_TIMEOUT)
            if dt is not None:
                break
            if time.time() - t_blk > (FIRST_TIMEOUT if first else NEXT_TIMEOUT):
                sys.exit(f"block {addr:#x}: refused {tries} times over {time.time() - t_blk:.1f} s; errors {res_err}")
            time.sleep(0.05)
        if tries > 1:
            print(f"block {addr:#x}: accepted on try {tries} after {time.time() - t_blk:.2f} s")
        if first:
            print(f"first block (erase) took {dt:.2f} s")
            first = False
        sent += 1
    print(f"{sent} blocks in {time.time() - t_all:.1f} s; booting")
    write_report(dev, b"\xFF\xFF\xFF" + bytes(BLOCK + 61), NEXT_TIMEOUT)
    k32.CloseHandle(dev)


if __name__ == "__main__":
    main()
