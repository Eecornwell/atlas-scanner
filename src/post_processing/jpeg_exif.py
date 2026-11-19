#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Orion. All rights reserved.
#
# Description: Minimal EXIF support for the JPEG training images, with no
# third-party dependency. Reads the capture settings from a camera JPEG (an
# X5 .insp is a JPEG) and writes them into the APP1 Exif segment of a new
# JPEG: exposure time, f-number, ISO, exposure bias, exposure program and
# mode, white balance, capture time, make and model. Trainers that model
# per-image exposure (PPISP-style) read these tags.

import struct
from fractions import Fraction
from pathlib import Path

# Tags read from a source file and written to the training images.
_IFD0 = {0x010F: 'Make', 0x0110: 'Model'}
_EXIF_IFD = {
    0x829A: 'ExposureTime',        # RATIONAL, seconds
    0x829D: 'FNumber',             # RATIONAL
    0x8822: 'ExposureProgram',     # SHORT
    0x8827: 'ISOSpeedRatings',     # SHORT (PhotographicSensitivity)
    0x9003: 'DateTimeOriginal',    # ASCII "YYYY:MM:DD HH:MM:SS"
    0x9204: 'ExposureBiasValue',   # SRATIONAL, EV
    0x9291: 'SubsecTimeOriginal',  # ASCII
    0xA402: 'ExposureMode',        # SHORT
    0xA403: 'WhiteBalance',        # SHORT
}
_TAG_OF = {name: tag for table in (_IFD0, _EXIF_IFD) for tag, name in table.items()}
_EXIF_IFD_POINTER = 0x8769
_EXIF_VERSION = 0x9000

_ASCII, _SHORT, _LONG, _RATIONAL, _UNDEFINED, _SRATIONAL = 2, 3, 4, 5, 7, 10
_TYPE_SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 7: 1, 8: 2, 9: 4, 10: 8}
_TYPE_OF = {
    'Make': _ASCII, 'Model': _ASCII, 'DateTimeOriginal': _ASCII, 'SubsecTimeOriginal': _ASCII,
    'ExposureTime': _RATIONAL, 'FNumber': _RATIONAL, 'ExposureBiasValue': _SRATIONAL,
    'ExposureProgram': _SHORT, 'ISOSpeedRatings': _SHORT, 'ExposureMode': _SHORT,
    'WhiteBalance': _SHORT,
}

TAG_NAMES = tuple(_TYPE_OF)


# ── reading ───────────────────────────────────────────────────────────────────

def _exif_tiff_block(data):
    """TIFF block of the first APP1 Exif segment of a JPEG, or None."""
    if data[:2] != b'\xff\xd8':
        return None
    i = 2
    while i + 4 <= len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker == 0xFF:                      # fill byte
            i += 1
            continue
        if marker in (0xD9, 0xDA):              # EOI / start of scan: no more headers
            return None
        if marker == 0x01 or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        seglen = struct.unpack('>H', data[i + 2:i + 4])[0]
        if marker == 0xE1 and data[i + 4:i + 10] == b'Exif\x00\x00':
            return data[i + 10:i + 2 + seglen]
        i += 2 + seglen
    return None


def _decode(typ, count, raw, endian):
    if typ == _ASCII:
        return raw.split(b'\x00', 1)[0].decode('ascii', 'replace').strip()
    if typ in (_RATIONAL, _SRATIONAL):
        f = 'I' if typ == _RATIONAL else 'i'
        v = struct.unpack(endian + f * (2 * count), raw)
        vals = [Fraction(v[2 * k], v[2 * k + 1]) if v[2 * k + 1] else None for k in range(count)]
    elif typ in (_SHORT, _LONG, 8, 9):
        f = {_SHORT: 'H', _LONG: 'I', 8: 'h', 9: 'i'}[typ]
        vals = list(struct.unpack(endian + f * count, raw))
    else:
        return raw
    return vals[0] if count == 1 else tuple(vals)


def _read_ifd(tiff, offset, endian):
    """{tag: value} of the IFD at `offset` in the TIFF block."""
    out = {}
    if offset + 2 > len(tiff):
        return out
    n = struct.unpack(endian + 'H', tiff[offset:offset + 2])[0]
    for k in range(n):
        e = offset + 2 + 12 * k
        if e + 12 > len(tiff):
            break
        tag, typ, count = struct.unpack(endian + 'HHI', tiff[e:e + 8])
        size = _TYPE_SIZE.get(typ)
        if size is None or count == 0:
            continue
        nbytes = size * count
        if nbytes <= 4:
            raw = tiff[e + 8:e + 8 + nbytes]
        else:
            ptr = struct.unpack(endian + 'I', tiff[e + 8:e + 12])[0]
            raw = tiff[ptr:ptr + nbytes]
        if len(raw) == nbytes:
            out[tag] = _decode(typ, count, raw, endian)
    return out


def read_exif(source, max_bytes=1 << 20):
    """Capture settings from the EXIF of a JPEG file (path) or JPEG bytes.

    Returns {name: value} for the names in TAG_NAMES that are present:
    rationals as fractions.Fraction, integers as int, text as str. Returns {}
    if there is no readable EXIF. Only the first `max_bytes` of a file are
    read (the Exif segment sits at the start; .insp files are large)."""
    if isinstance(source, (bytes, bytearray)):
        data = bytes(source)
    else:
        try:
            with open(source, 'rb') as f:
                data = f.read(max_bytes)
        except OSError:
            return {}
    tiff = _exif_tiff_block(data)
    if not tiff or len(tiff) < 8 or tiff[:2] not in (b'II', b'MM'):
        return {}
    endian = '<' if tiff[:2] == b'II' else '>'
    try:
        ifd0 = _read_ifd(tiff, struct.unpack(endian + 'I', tiff[4:8])[0], endian)
        exif_ifd = {}
        if isinstance(ifd0.get(_EXIF_IFD_POINTER), int):
            exif_ifd = _read_ifd(tiff, ifd0[_EXIF_IFD_POINTER], endian)
    except struct.error:
        return {}
    out = {}
    for table, ifd in ((_IFD0, ifd0), (_EXIF_IFD, exif_ifd)):
        for tag, name in table.items():
            v = ifd.get(tag)
            if isinstance(v, tuple):            # e.g. ISO written with count > 1
                v = v[0] if v else None
            if v is None or v == '':
                continue
            if _TYPE_OF[name] in (_RATIONAL, _SRATIONAL) and not isinstance(v, Fraction):
                v = Fraction(v)
            if _TYPE_OF[name] == _SHORT and not isinstance(v, int):
                continue
            out[name] = v
    return out


# ── writing ───────────────────────────────────────────────────────────────────

def _rational(x, signed):
    fr = x if isinstance(x, Fraction) else Fraction(x).limit_denominator(1000000)
    lim = 0x7FFFFFFF if signed else 0xFFFFFFFF
    while abs(fr.numerator) > lim or fr.denominator > lim:
        fr = Fraction(round(fr.numerator / 2), max(1, round(fr.denominator / 2)))
    return struct.pack('<ii' if signed else '<II', fr.numerator, fr.denominator)


def _entry(name, value):
    """(tag, type, count, payload) for one tag, or None if the value is unusable."""
    typ = _TYPE_OF[name]
    try:
        if typ == _ASCII:
            payload = str(value).encode('ascii', 'replace') + b'\x00'
            return _TAG_OF[name], typ, len(payload), payload
        if typ == _SHORT:
            return _TAG_OF[name], typ, 1, struct.pack('<H', max(0, min(int(value), 0xFFFF)))
        if typ == _RATIONAL and float(value) < 0:
            return None
        return _TAG_OF[name], typ, 1, _rational(value, typ == _SRATIONAL)
    except (TypeError, ValueError, OverflowError, ZeroDivisionError):
        return None


def _ifd_bytes(entries, start):
    """IFD (little-endian) placed at TIFF offset `start`: count, sorted
    12-byte entries, next-IFD 0, then the values that don't fit in 4 bytes."""
    entries = sorted(entries)
    data_off = start + 2 + 12 * len(entries) + 4
    head, data = struct.pack('<H', len(entries)), b''
    for tag, typ, count, payload in entries:
        if len(payload) <= 4:
            head += struct.pack('<HHI', tag, typ, count) + payload.ljust(4, b'\x00')
        else:
            head += struct.pack('<HHII', tag, typ, count, data_off + len(data))
            data += payload + (b'\x00' if len(payload) % 2 else b'')
    return head + struct.pack('<I', 0) + data


def _exif_pointer(offset):
    return _EXIF_IFD_POINTER, _LONG, 1, struct.pack('<I', offset)


def exif_segment(tags):
    """APP1 Exif segment (marker included) for {name: value}, skipping names
    it doesn't know and unusable values; None if nothing is left."""
    ifd0, exif = [], []
    for name, value in tags.items():
        if name not in _TYPE_OF or value is None or value == '':
            continue
        e = _entry(name, value)
        if e is not None:
            (ifd0 if name in _IFD0.values() else exif).append(e)
    if not ifd0 and not exif:
        return None
    exif.append((_EXIF_VERSION, _UNDEFINED, 4, b'0232'))
    # The IFD0 size doesn't depend on the pointer value, so measure it first.
    exif_start = 8 + len(_ifd_bytes(ifd0 + [_exif_pointer(0)], 8))
    tiff = (b'II*\x00' + struct.pack('<I', 8)
            + _ifd_bytes(ifd0 + [_exif_pointer(exif_start)], 8)
            + _ifd_bytes(exif, exif_start))
    body = b'Exif\x00\x00' + tiff
    return b'\xff\xe1' + struct.pack('>H', len(body) + 2) + body


def insert_exif(jpeg, tags):
    """JPEG bytes with an Exif APP1 segment for `tags` right after SOI.
    Drops the JFIF APP0 and any existing Exif APP1 (Exif files carry APP1
    first); every other segment and the image data are kept as they are."""
    seg = exif_segment(tags)
    if seg is None:
        return jpeg
    if jpeg[:2] != b'\xff\xd8':
        raise ValueError('not a JPEG')
    i, kept = 2, []
    while i + 4 <= len(jpeg) and jpeg[i] == 0xFF and jpeg[i + 1] not in (0xDA, 0xD9):
        if jpeg[i + 1] == 0xFF:                 # fill byte
            i += 1
            continue
        seglen = struct.unpack('>H', jpeg[i + 2:i + 4])[0]
        marker, body = jpeg[i + 1], jpeg[i + 4:i + 2 + seglen]
        if not ((marker == 0xE0 and body[:5] == b'JFIF\x00') or
                (marker == 0xE1 and body[:6] == b'Exif\x00\x00')):
            kept.append(jpeg[i:i + 2 + seglen])
        i += 2 + seglen
    return jpeg[:2] + seg + b''.join(kept) + jpeg[i:]


def encode_jpeg(img, tags=None, quality=100):
    """Encode a BGR/grey image as JPEG (4:4:4 chroma when OpenCV supports it)
    with an Exif segment for `tags`."""
    import cv2
    params = [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
    sampling = getattr(cv2, 'IMWRITE_JPEG_SAMPLING_FACTOR', None)
    s444 = getattr(cv2, 'IMWRITE_JPEG_SAMPLING_FACTOR_444', None)
    if sampling is not None and s444 is not None:
        params += [sampling, s444]
    ok, buf = cv2.imencode('.jpg', img, params)
    if not ok:
        raise ValueError('JPEG encoding failed')
    data = buf.tobytes()
    return insert_exif(data, tags) if tags else data


def write_jpeg(path, img, tags=None, quality=100):
    """Write `img` to `path` as JPEG with Exif `tags` (see encode_jpeg)."""
    Path(path).write_bytes(encode_jpeg(img, tags, quality))
