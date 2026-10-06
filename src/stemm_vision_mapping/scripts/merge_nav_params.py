#!/usr/bin/env python3
"""把生产 Nav2 参数与视觉叠加参数深合并成一份（Nav2 只接受单个 params_file）。"""
import sys, yaml, copy

base_p, overlay_p, out_p = sys.argv[1], sys.argv[2], sys.argv[3]
base = yaml.safe_load(open(base_p, encoding='utf-8')) or {}
over = yaml.safe_load(open(overlay_p, encoding='utf-8')) or {}

def deep_merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = copy.deepcopy(v)
    return dst

merged = deep_merge(copy.deepcopy(base), over)
with open(out_p, 'w', encoding='utf-8') as f:
    yaml.safe_dump(merged, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

# 校验关键项
lc = merged.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {})
gc = merged.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {})
print(f'  本地图层: {lc.get("plugins")}')
print(f'  全局图层: {gc.get("plugins")}')
print(f'  本地激光源: {lc.get("voxel_layer", {}).get("observation_sources")}')
print(f'  视觉体素源: {lc.get("vision_voxel_layer", {}).get("observation_sources")}')
print(f'  视觉静态层: {gc.get("vision_static_layer", {}).get("map_topic")}')
print(f'  已写出: {out_p}')
