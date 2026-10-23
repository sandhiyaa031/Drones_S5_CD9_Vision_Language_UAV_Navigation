#!/usr/bin/env python3
"""Convert an XWD window dump (from `xwd -id WINDOW`) to PNG.

Usage: xwd2png.py IN.xwd OUT.png      (32-bit ZPixmap dumps only)
"""
import struct
import sys

import cv2
import numpy as np


def main():
    data = open(sys.argv[1], 'rb').read()
    (header_size, _ver, fmt, depth, width, height, _xoff, byte_order,
     _unit, _bit_order, _pad, bpp, bytes_per_line, _vis, red, green, blue,
     _bits_rgb, _entries, ncolors) = struct.unpack('>20I', data[:80])
    offset = header_size + ncolors * 12
    # depth-24 visuals are stored in 32-bit units (bits_per_pixel may say 24)
    if fmt != 2 or bytes_per_line != width * 4:
        raise SystemExit(f'unsupported dump: bpp={bpp} format={fmt}')
    raw = np.frombuffer(data, np.uint8, offset=offset,
                        count=bytes_per_line * height)
    img = raw.reshape(height, bytes_per_line // 4, 4)[:, :width]
    # little-endian pixels are stored B, G, R, X
    out = img[:, :, :3] if byte_order == 0 else img[:, :, 1:4][:, :, ::-1]
    cv2.imwrite(sys.argv[2], np.ascontiguousarray(out))


if __name__ == '__main__':
    main()
