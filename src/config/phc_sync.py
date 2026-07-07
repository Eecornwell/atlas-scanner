#!/usr/bin/env python3
"""
Continuously sync /dev/ptp0 to CLOCK_REALTIME by calling clock_settime
at 10 Hz. The igc PHC runs ~33000ppm fast so frequency adjustment alone
(phc2sys) can't keep up when delay measurement is unavailable. Repeated
hard steps at 10Hz keep the PHC within ~3ms of system time at all times.

This is intentionally simple — no servo, no frequency learning. The MID360
slaves its hardware clock to our PHC via PTP peer-delay, so as long as the
PHC is within a few ms of system time the LiDAR timestamps will match
t_before to within that same margin.
"""
import ctypes
import ctypes.util
import time
import sys
import os
import signal

CLOCK_REALTIME = 0
PHC_CLOCKID_MAGIC = 0xC0000000  # ~(clockid) for PHC fd-based clock IDs

def open_phc(path='/dev/ptp0'):
    fd = os.open(path, os.O_RDWR)
    # Convert fd to clockid: (~fd << 3) | 3  (kernel formula)
    clockid = (~fd << 3) | 3
    return fd, clockid

def make_timespec():
    class timespec(ctypes.Structure):
        _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]
    return timespec

def main():
    libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)
    timespec = make_timespec()

    try:
        fd, phc_clockid = open_phc('/dev/ptp0')
    except OSError as e:
        print(f'Cannot open /dev/ptp0: {e}', file=sys.stderr)
        sys.exit(1)

    print(f'phc_sync: opened /dev/ptp0 as clockid {phc_clockid:#x}, syncing at 10Hz')
    sys.stdout.flush()

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    last_log = 0.0
    while running:
        t0 = time.clock_gettime(time.CLOCK_REALTIME)
        ts = timespec(tv_sec=int(t0), tv_nsec=int((t0 % 1) * 1e9))
        ret = libc.clock_settime(phc_clockid, ctypes.byref(ts))
        if ret != 0:
            err = ctypes.get_errno()
            print(f'phc_sync: clock_settime failed: errno={err}', file=sys.stderr)
            sys.exit(1)

        now = time.monotonic()
        if now - last_log >= 10.0:
            # Read back PHC to confirm offset
            ts_read = timespec()
            libc.clock_gettime(phc_clockid, ctypes.byref(ts_read))
            phc_t = ts_read.tv_sec + ts_read.tv_nsec * 1e-9
            sys_t = time.clock_gettime(time.CLOCK_REALTIME)
            offset_ms = (sys_t - phc_t) * 1000
            print(f'phc_sync: offset={offset_ms:+.2f}ms  PHC={phc_t:.3f}  sys={sys_t:.3f}')
            sys.stdout.flush()
            last_log = now

        time.sleep(0.1)  # 10 Hz

    os.close(fd)
    print('phc_sync: stopped')

if __name__ == '__main__':
    main()
