#!/usr/bin/env python3
"""生成对照图：灰阶=激光图（深灰占据/近白空闲/中灰未知），红色=视觉新增障碍。
用法：make_highlight.py <laser.yaml> <fused.yaml> <out.png>
"""
import os, re, struct, sys, zlib
import numpy as np


def read_pgm(p):
    d = open(p, 'rb').read()
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(d[m.end():m.end()+w*h], dtype=np.uint8).reshape(h, w)


def read_yaml(p):
    t = open(p, encoding='utf-8').read()
    return (float(re.search(r'resolution:\s*([\d.]+)', t).group(1)),
            *[float(x) for x in re.search(r'origin:\s*\[([^\]]+)\]', t).group(1).split(',')[:2]],
            re.search(r'image:\s*(\S+)', t).group(1))


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[i].tobytes() for i in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) \
        + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b'')
    open(path, 'wb').write(png)


def main():
    ly, fy, out = sys.argv[1], sys.argv[2], sys.argv[3]
    lres, lox, loy, limg = read_yaml(ly)
    fres, fox, foy, fimg = read_yaml(fy)
    ls = read_pgm(os.path.join(os.path.dirname(ly), limg))
    fs = read_pgm(os.path.join(os.path.dirname(fy), fimg))
    lh, lw = ls.shape; fh, fw = fs.shape
    ls = np.flipud(ls); fs = np.flipud(fs)
    rgb = np.zeros((fh, fw, 3), dtype=np.uint8)
    rgb[..., :] = (128, 128, 128)
    rgb[fs > 240] = (245, 245, 245)
    rgb[fs < 100] = (40, 40, 40)
    added = 0
    for r in range(fh):
        if not (fs[r] < 100).any():
            continue
        lr = int((foy + (fh-1-r)*fres + fres/2 - loy) / lres)
        if not (0 <= lr < lh):
            continue
        for c in np.flatnonzero(fs[r] < 100):
            lc = int((fox + c*fres + fres/2 - lox) / lres)
            if not (0 <= lc < lw and ls[lr, lc] < 100):
                rgb[r, c] = (230, 30, 30); added += 1
    write_png(out, rgb)
    print(f'  高亮图: {out}   视觉新增(红)={added} 格')


if __name__ == '__main__':
    main()
