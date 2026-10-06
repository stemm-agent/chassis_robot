#!/usr/bin/env python3
"""视觉障碍格可信度校验 + 保守过滤（孤立误检剔除）。

输入：激光图 yaml、视觉障碍图 yaml（两者均为 nav2 标准 pgm/yaml）
输出：
  1) 控制台报告：新增格到最近激光障碍的距离分布、连通域大小分布
  2) <前缀>_conservative.pgm/.yaml ：剔除孤立小连通域后的视觉障碍图
  3) <前缀>_conservative_fused.pgm/.yaml ：激光图 ∪ 保守视觉障碍
用法：grid_confidence.py <laser.yaml> <vision.yaml> <输出前缀> [--min-cells 2]
"""
import argparse, os, re
from collections import deque
import numpy as np


def read_pgm(p):
    d = open(p, 'rb').read()
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(d[m.end():m.end()+w*h], dtype=np.uint8).reshape(h, w)


def read_yaml(p):
    t = open(p, encoding='utf-8').read()
    res = float(re.search(r'resolution:\s*([\d.]+)', t).group(1))
    o = re.search(r'origin:\s*\[([^\]]+)\]', t).group(1).split(',')
    img = re.search(r'image:\s*(\S+)', t).group(1)
    return res, float(o[0]), float(o[1]), img


def load(path):
    res, ox, oy, img = read_yaml(path)
    a = read_pgm(os.path.join(os.path.dirname(path), img))
    return res, ox, oy, a


def to_world_index(a, res, ox, oy, ref_res, ref_ox, ref_oy, W, H):
    """把图 a 的每格映射到参考栅格索引；返回 (rows, cols, vals)"""
    h, w = a.shape
    rows, cols = np.mgrid[0:h, 0:w]
    wy = oy + (h - 1 - rows) * res + res / 2.0      # PGM 行 0 在上
    wx = ox + cols * res + res / 2.0
    rr = np.floor((wy - ref_oy) / ref_res).astype(np.int64)
    cc = np.floor((wx - ref_ox) / ref_res).astype(np.int64)
    ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
    return rr[ok], cc[ok], a[rows[ok], cols[ok]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('laser_yaml'); ap.add_argument('vision_yaml'); ap.add_argument('out_prefix')
    ap.add_argument('--min-cells', type=int, default=2, help='连通域最小格数，小于则视为孤立误检')
    a = ap.parse_args()

    lres, lox, loy, limg = load(a.laser_yaml)
    vres, vox, voy, vimg = load(a.vision_yaml)
    lh, lw = limg.shape
    vh, vw = vimg.shape
    print(f'  激光图 {lw}x{lh} @{lres} ({lox:.2f},{loy:.2f})   视觉障碍图 {vw}x{vh} @{vres} ({vox:.2f},{voy:.2f})')

    # 统一到 0.05 m 参考栅格（用激光图为基准，覆盖并集范围）
    res = lres
    x0 = min(lox, vox); y0 = min(loy, voy)
    x1 = max(lox + lw*res, vox + vw*vres); y1 = max(loy + lh*res, voy + vh*vres)
    W = int(round((x1 - x0)/res)) + 2; H = int(round((y1 - y0)/res)) + 2
    print(f'  参考栅格 {W}x{H} @{res} 原点 ({x0:.2f},{y0:.2f})')

    L = np.zeros((H, W), dtype=np.uint8)      # 0 未知 1 空闲 2 占据
    rr, cc, vv = to_world_index(limg, lres, lox, loy, res, x0, y0, W, H)
    L[rr[vv > 240], cc[vv > 240]] = 1
    L[rr[vv < 100], cc[vv < 100]] = 2
    V = np.zeros((H, W), dtype=bool)
    rr, cc, vv = to_world_index(vimg, vres, vox, voy, res, x0, y0, W, H)
    V[rr[vv < 100], cc[vv < 100]] = True
    laser_occ = (L == 2)
    print(f'  栅格化：激光占据 {int(laser_occ.sum())}   视觉障碍 {int(V.sum())}   其中与激光重合 {int((V & laser_occ).sum())}')

    # ---- 1) 到最近激光障碍的距离（多源 BFS）----
    INF = 10**9
    dist = np.full((H, W), INF, dtype=np.int32)
    dq = deque()
    ys, xs = np.nonzero(laser_occ)
    for y, x in zip(ys.tolist(), xs.tolist()):
        dist[y, x] = 0; dq.append((y, x))
    while dq:
        y, x = dq.popleft()
        d = dist[y, x] + 1
        for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
            if 0 <= ny < H and 0 <= nx < W and dist[ny, nx] > d:
                dist[ny, nx] = d; dq.append((ny, nx))
    added = V & ~laser_occ
    d_add = dist[added] * res
    print('\n  === 视觉新增格 到 最近激光障碍 的距离分布 ===')
    bins = [0.0, 0.10, 0.20, 0.30, 0.50, 0.80, 1.50, 1e9]
    labels = ['≤0.10 m', '0.10–0.20', '0.20–0.30', '0.30–0.50', '0.50–0.80', '0.80–1.50', '>1.50 m']
    tot = max(1, d_add.size)
    for i, lb in enumerate(labels):
        n = int(((d_add >= bins[i]) & (d_add < bins[i+1])).sum())
        print(f'    {lb:>10}: {n:6d}  ({100*n/tot:5.1f}%)  {"█"*int(40*n/tot)}')
    if d_add.size:
        print(f'    中位数 {np.median(d_add):.2f} m   均值 {d_add.mean():.2f} m')

    # ---- 2) 连通域分析（剔除孤立误检）----
    lbl = np.zeros((H, W), dtype=np.int32)
    cur = 0
    sizes = {}
    vv_ys, vv_xs = np.nonzero(V)
    for y0_, x0_ in zip(vv_ys.tolist(), vv_xs.tolist()):
        if lbl[y0_, x0_]:
            continue
        cur += 1
        q = deque([(y0_, x0_)]); lbl[y0_, x0_] = cur; n = 0
        while q:
            y, x = q.popleft(); n += 1
            for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
                if 0 <= ny < H and 0 <= nx < W and V[ny, nx] and not lbl[ny, nx]:
                    lbl[ny, nx] = cur; q.append((ny, nx))
        sizes[cur] = n
    hist = {}
    for s in sizes.values():
        k = '1 格' if s == 1 else ('2 格' if s == 2 else ('3–5 格' if s <= 5 else ('6–20 格' if s <= 20 else '>20 格')))
        hist[k] = hist.get(k, 0) + 1
    print(f'\n  === 视觉障碍连通域统计（共 {cur} 个域，{int(V.sum())} 格）===')
    for k in ['1 格', '2 格', '3–5 格', '6–20 格', '>20 格']:
        if k in hist:
            print(f'    {k:>8}: {hist[k]:4d} 个域')

    keep = np.zeros((H, W), dtype=bool)
    for i in range(1, cur+1):
        if sizes[i] >= a.min_cells:
            keep |= (lbl == i)
    dropped = int((V & ~keep).sum())
    print(f'\n  保守过滤（保留连通域 ≥{a.min_cells} 格）：保留 {int(keep.sum())} 格，剔除 {dropped} 格（{100*dropped/max(1,int(V.sum())):.1f}%）')

    # ---- 3) 写出保守版（回到视觉图自身坐标系）----
    def write_grid(mask_full, path, ref_ox, ref_oy):
        rows, cols = np.nonzero(mask_full)
        if rows.size == 0:
            print('  (空图，跳过)'); return
        minr, maxr = rows.min(), rows.max(); minc, maxc = cols.min(), cols.max()
        h, w = maxr-minr+1, maxc-minc+1
        img = np.full((h, w), 205, dtype=np.uint8)
        img[rows-minr, cols-minc] = 0
        img = np.flipud(img)
        ox_ = ref_ox + minc*res; oy_ = ref_oy + minr*res
        with open(path+'.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h)); f.write(img.tobytes())
        with open(path+'.yaml', 'w', encoding='utf-8') as f:
            f.write(f'image: {os.path.basename(path)}.pgm\nmode: trinary\nresolution: {res}\n'
                    f'origin: [{ox_:.6f}, {oy_:.6f}, 0.0]\nnegate: 0\n'
                    f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
        print(f'  已写出 {path}.pgm/.yaml  {w}x{h} 原点 ({ox_:.2f},{oy_:.2f})')

    write_grid(keep, a.out_prefix + '_conservative', x0, y0)

    # 融合：激光(占据/空闲/未知) + 保守视觉障碍（只增）
    fused = np.full((H, W), 205, dtype=np.uint8)
    fused[L == 1] = 254
    fused[L == 2] = 0
    fused[keep] = 0
    added_kept = int((keep & ~laser_occ).sum())
    print(f'  保守融合图：视觉净增 {added_kept} 格（原始为 {int(added.sum())} 格）')
    write_grid(fused == 0, a.out_prefix + '_conservative_fused_onlyocc', x0, y0)
    # 写完整三态融合图
    pw = a.out_prefix + '_conservative_fused'
    with open(pw+'.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(np.flipud(fused).tobytes())
    with open(pw+'.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(pw)}.pgm\nmode: trinary\nresolation: {res}\n'
                .replace('resolation', 'resolution') +
                f'origin: [{x0:.6f}, {y0:.6f}, 0.0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'  已写出 {pw}.pgm/.yaml  {W}x{H}')


if __name__ == '__main__':
    main()
