#!/usr/bin/env python3
"""从 rtabmap-export 导出的 3D 点云（闭环修正后的位姿）投影出 2D 障碍栅格。

用法：cloud_to_grid.py <cloud.ply> <输出前缀> [--res 0.05] [--zmin 0.05] [--zmax 1.80] [--min-pts 1]
产物：<前缀>.pgm / <前缀>.yaml（0=占据 205=未知 254=空闲 的 nav2 标准格式）
"""
import argparse, os, struct, zlib
import numpy as np

DT = {'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8',
      'uchar': 'u1', 'uint8': 'u1', 'char': 'i1', 'int8': 'i1',
      'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
      'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2'}


def read_ply(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise SystemExit('不是 PLY 文件')
        fmt, cur, nvert, props = None, None, 0, []
        while True:
            line = f.readline()
            if not line:
                raise SystemExit('PLY 头未结束')
            s = line.strip()
            if s.startswith(b'format'):
                fmt = s.split()[1].decode()
            elif s.startswith(b'element'):
                cur = s.split()[1].decode()
                if cur == 'vertex':
                    nvert = int(s.split()[2])
            elif s.startswith(b'property') and cur == 'vertex':
                p = s.split()
                if p[1] != b'list':
                    props.append((p[2].decode(), p[1].decode()))
            elif s.startswith(b'end_header'):
                break
        dt = np.dtype([(n, DT[t]) for n, t in props])
        if fmt == 'binary_little_endian':
            buf = f.read(nvert * dt.itemsize)
            data = np.frombuffer(buf, dtype=dt)
        elif fmt == 'ascii':
            data = np.loadtxt(f, dtype=dt, max_rows=nvert, ndmin=1)
        else:
            raise SystemExit(f'暂不支持的 PLY 格式: {fmt}')
    return data, fmt, nvert


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[i].tobytes() for i in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw, 6))
    png += chunk(b'IEND', b'')
    open(path, 'wb').write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply'); ap.add_argument('out_prefix')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--zmin', type=float, default=0.05)
    ap.add_argument('--zmax', type=float, default=1.80)
    ap.add_argument('--min-pts', type=int, default=1)
    a = ap.parse_args()

    data, fmt, n = read_ply(a.ply)
    names = data.dtype.names
    print(f'  PLY: {n} 点, 格式 {fmt}, 字段 {names}')
    x, y, z = data['x'].astype(np.float64), data['y'].astype(np.float64), data['z'].astype(np.float64)
    keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > a.zmin) & (z < a.zmax)
    print(f'  高度带 [{a.zmin},{a.zmax}] m 内: {int(keep.sum())} 点（占 {100*keep.sum()/max(1,n):.1f}%）')
    ix = np.floor(x[keep] / a.res).astype(np.int64)
    iy = np.floor(y[keep] / a.res).astype(np.int64)
    OFF, base = 1 << 20, 2 << 20
    key = (ix + OFF) * base + (iy + OFF)
    uniq, cnt = np.unique(key, return_counts=True)
    good = uniq[cnt >= a.min_pts]
    gx = good // base - OFF
    gy = good % base - OFF
    print(f'  投影栅格: {len(good)} 格（{a.res} m，单格≥{a.min_pts} 点）')

    minx, maxx, miny, maxy = gx.min(), gx.max(), gy.min(), gy.max()
    w, h = int(maxx - minx + 1), int(maxy - miny + 1)
    img = np.full((h, w), 205, dtype=np.uint8)
    img[(gy - miny), (gx - minx)] = 0
    img = np.flipud(img)                     # PGM 行 0 在上
    with open(a.out_prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h)); f.write(img.tobytes())
    ox, oy = minx * a.res, miny * a.res
    with open(a.out_prefix + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(a.out_prefix)}.pgm\nmode: trinary\n'
                f'resolution: {a.res}\norigin: [{ox:.6f}, {oy:.6f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'  已写出: {a.out_prefix}.pgm / .yaml   尺寸 {w}x{h}  原点 ({ox:.2f},{oy:.2f})')


if __name__ == '__main__':
    main()
