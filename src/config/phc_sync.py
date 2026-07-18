#!/usr/bin/env python3
"""
PI servo to discipline /dev/ptp0 to CLOCK_REALTIME.

The igc PHC drifts ~33000ppm so hard clock_settime steps at 10Hz cause the
MID360's PTP peer-delay loop to constantly re-acquire lock, producing jitter
in LiDAR header.stamp.  A PI servo slews the PHC smoothly, keeping the
MID360's PTP offset < 1ms in steady state.

Control law (runs at 20Hz):
  error  = sys_time - phc_time          (seconds)
  freq   += Ki * error                  (integrator, ppm)
  adj    = Kp * error + freq            (total adjustment)
  Apply adj via clock_adjtime(ADJ_SETOFFSET) for large errors,
  ADJ_FREQUENCY for small tracking errors.

Kp=0.7, Ki=0.3 are conservative values that converge in ~5s without
overshoot on a 33000ppm-fast clock.
"""
import ctypes
import ctypes.util
import time
import sys
import os
import signal
import struct

CLOCK_REALTIME = 0
ADJ_OFFSET     = 0x0001
ADJ_FREQUENCY  = 0x0002
ADJ_SETOFFSET  = 0x0100

# PI gains — tune if convergence is too slow or oscillates
Kp = 0.7
Ki = 0.3
LOOP_HZ = 20.0
STEP_THRESHOLD_S = 0.128  # step directly if error > 128ms (initial sync)


def open_phc(path='/dev/ptp0'):
    fd = os.open(path, os.O_RDWR)
    clockid = (~fd << 3) | 3
    return fd, clockid


def make_timespec_type():
    class timespec(ctypes.Structure):
        _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]
    return timespec


def make_timex_type():
    """struct timex for clock_adjtime — only the fields we use."""
    class timex(ctypes.Structure):
        _fields_ = [
            ('modes',     ctypes.c_uint),
            ('offset',    ctypes.c_long),   # ns (ADJ_OFFSET) or tv_sec (ADJ_SETOFFSET)
            ('freq',      ctypes.c_long),   # ppm << 16
            ('maxerror',  ctypes.c_long),
            ('esterror',  ctypes.c_long),
            ('status',    ctypes.c_int),
            ('constant',  ctypes.c_long),
            ('precision', ctypes.c_long),
            ('tolerance', ctypes.c_long),
            ('time',      ctypes.c_long * 2),  # struct timeval (tv_sec, tv_usec)
            ('tick',      ctypes.c_long),
            ('ppsfreq',   ctypes.c_long),
            ('jitter',    ctypes.c_long),
            ('shift',     ctypes.c_int),
            ('stabil',    ctypes.c_long),
            ('jitcnt',    ctypes.c_long),
            ('calcnt',    ctypes.c_long),
            ('errcnt',    ctypes.c_long),
            ('stbcnt',    ctypes.c_long),
            ('tai',       ctypes.c_int),
            ('_pad',      ctypes.c_int * 11),
        ]
    return timex


def main():
    libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)
    timespec = make_timespec_type()
    timex    = make_timex_type()

    # clock_adjtime syscall number (x86_64 = 305)
    NR_clock_adjtime = 305
    syscall = libc.syscall
    syscall.restype = ctypes.c_long

    try:
        fd, phc_clockid = open_phc('/dev/ptp0')
    except OSError as e:
        print(f'Cannot open /dev/ptp0: {e}', file=sys.stderr)
        sys.exit(1)

    print(f'phc_sync: PI servo on /dev/ptp0 (clockid={phc_clockid:#x}) at {LOOP_HZ:.0f}Hz')
    sys.stdout.flush()

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    integrator = 0.0  # accumulated frequency correction (ppm)
    last_log   = 0.0
    dt         = 1.0 / LOOP_HZ

    while running:
        t_sys = time.clock_gettime(time.CLOCK_REALTIME)

        ts_phc = timespec()
        if libc.clock_gettime(phc_clockid, ctypes.byref(ts_phc)) != 0:
            print('phc_sync: clock_gettime(PHC) failed', file=sys.stderr)
            sys.exit(1)
        t_phc = ts_phc.tv_sec + ts_phc.tv_nsec * 1e-9

        error = t_sys - t_phc  # positive = PHC is behind

        if abs(error) > STEP_THRESHOLD_S:
            # Large error: hard step via clock_settime
            ts = timespec(tv_sec=int(t_sys), tv_nsec=int((t_sys % 1) * 1e9))
            libc.clock_settime(phc_clockid, ctypes.byref(ts))
            integrator = 0.0
            now = time.monotonic()
            print(f'phc_sync: STEP  error={error*1000:+.1f}ms', flush=True)
            last_log = now
        else:
            # Small error: PI servo via clock_adjtime ADJ_OFFSET + ADJ_FREQUENCY
            integrator += Ki * error * dt
            integrator  = max(-500e-6, min(500e-6, integrator))  # clamp ±500ppm
            freq_corr   = Kp * error + integrator  # seconds/second

            tx = timex()
            tx.modes  = ADJ_OFFSET | ADJ_FREQUENCY
            tx.offset = int(error * 1e9)           # nanoseconds
            tx.freq   = int(freq_corr * (1 << 16) * 1e6)  # ppm << 16
            syscall(NR_clock_adjtime, phc_clockid, ctypes.byref(tx))

        now = time.monotonic()
        if now - last_log >= 10.0:
            print(f'phc_sync: offset={error*1000:+.2f}ms  '
                  f'integrator={integrator*1e6:+.1f}ppm  '
                  f'PHC={t_phc:.3f}  sys={t_sys:.3f}', flush=True)
            last_log = now

        time.sleep(dt)

    os.close(fd)
    print('phc_sync: stopped')


if __name__ == '__main__':
    main()

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
