#!/usr/bin/env python3
"""把激光地图与深度障碍图融合成一张导航地图（纯 Python，无第三方依赖）。

规则（只增不减，安全）：
  - 以激光地图为基准（分辨率/坐标系一致或自动重采样到激光地图分辨率）
  - 深度障碍图里的障碍格 → 在融合图中标为占据（补上激光盲区：低矮/悬空/玻璃）
  - 深度图的未知格 **不覆盖** 激光图的任何信息（避免把空地抹掉）
用法：merge_maps.py <激光地图.yaml> <深度障碍图.yaml> [输出前缀]
"""
import sys, os, re
import numpy as np


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # 解析 P5 头（可能有注释）
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', data)
    if not m:
        raise SystemExit('不是 P5 PGM: ' + path)
    w, h, maxv = int(m.group(1)), int(m.group(2)), int(m.group(3))
    off = m.end()
    img = np.frombuffer(data[off:off + w * h], dtype=np.uint8).reshape(h, w)
    return img, w, h, maxv


def read_yaml(path):
    txt = open(path, encoding='utf-8').read()
    def g(key, default=None):
        mm = re.search(rf'{key}:\s*([^\s#]+)', txt)
        return mm.group(1) if mm else default
    res = float(g('resolution', '0.05'))
    org = re.search(r'origin:\s*\[([^\]]+)\]', txt)
    ox, oy = (float(org.group(1).split(',')[0]), float(org.group(1).split(',')[1])) if org else (0.0, 0.0)
    img = g('image')
    return res, ox, oy, img


def to_world_grid(img, res, ox, oy):
    """返回 (occupied_cells_world_set_as_index) —— 用世界坐标->索引的映射参数"""
    h, w = img.shape
    return dict(img=img, res=res, ox=ox, oy=oy, w=w, h=h)


def main():
    laser_yaml, vis_yaml = sys.argv[1], sys.argv[2]
    out_prefix = sys.argv[3] if len(sys.argv) > 3 else os.path.splitext(laser_yaml)[0] + '_fused'

    lres, lox, loy, limg_name = read_yaml(laser_yaml)
    limg, lw, lh, _ = read_pgm(os.path.join(os.path.dirname(laser_yaml), limg_name))
    vres, vox, voy, vimg_name = read_yaml(vis_yaml)
    vimg, vw, vh, _ = read_pgm(os.path.join(os.path.dirname(vis_yaml), vimg_name))

    print(f'激光图: {lw}x{lh} res={lres} origin=({lox:.2f},{loy:.2f})')
    print(f'深度图: {vw}x{vh} res={vres} origin=({vox:.2f},{voy:.2f})')

    # 统一到激光地图的分辨率；计算并集范围（世界坐标，米）
    def extent(res, ox, oy, w, h):
        return ox, oy, ox + w * res, oy + h * res
    lx0, ly0, lx1, ly1 = extent(lres, lox, loy, lw, lh)
    vx0, vy0, vx1, vy1 = extent(vres, vox, voy, vw, vh)
    x0, y0 = min(lx0, vx0), min(ly0, vy0)
    x1, y1 = max(lx1, vx1), max(ly1, vy1)
    W = int(round((x1 - x0) / lres))
    H = int(round((y1 - y0) / lres))
    print(f'融合图: {W}x{H} @ {lres} m, origin=({x0:.2f},{y0:.2f})')

    fused = np.full((H, W), 205, dtype=np.uint8)  # 205 = 未知

    def paste(src, res, ox, oy, overwrite_occupied_only=False):
        h, w = src.shape
        # 世界坐标 -> 融合图索引（行 0 = 最高 y）
        for row in range(h):
            wy = oy + (h - 1 - row) * res + res / 2.0
            fr = int((wy - y0) / lres)
            if fr < 0 or fr >= H:
                continue
            for col in range(w):
                v = src[row, col]
                if overwrite_occupied_only and v > 100:   # 深度图：只搬"占据"
                    pass
                wx = ox + col * res + res / 2.0
                fc = int((wx - x0) / lres)
                if fc < 0 or fc >= W:
                    continue
                fused[fr, fc] = v

    # 1) 先铺激光地图（含 free/occupied/unknown）
    paste(limg, lres, lox, loy)

    # 2) 再把深度障碍"只增"地叠上去
    added = 0
    vh_, vw_ = vimg.shape
    for row in range(vh_):
        wy = voy + (vh_ - 1 - row) * vres + vres / 2.0
        fr = int((wy - y0) / lres)
        if fr < 0 or fr >= H:
            continue
        for col in range(vw_):
            if vimg[row, col] >= 100:     # PGM 约定：0=占据(黑) 205=未知 254=空闲 → 只搬占据格
                continue
            wx = vox + col * vres + vres / 2.0
            fc = int((wx - x0) / lres)
            if 0 <= fc < W and fused[fr, fc] != 0:
                fused[fr, fc] = 0         # 0 = 占据（PGM 约定）
                added += 1
    print(f'深度新增障碍格（激光图原本不是障碍的）: {added}')

    pgm = out_prefix + '.pgm'
    yml = out_prefix + '.yaml'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(fused.tobytes())
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(pgm)}\n'
                f'mode: trinary\nresolution: {lres}\n'
                f'origin: [{x0:.6f}, {y0:.6f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'已写出: {pgm} / {yml}')


if __name__ == '__main__':
    main()
