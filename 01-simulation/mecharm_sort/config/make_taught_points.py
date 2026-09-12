#!/usr/bin/env python3
# 【讲解】从逆解生成一份"示教点"文件（仿真用），格式与真机示教产出的 taught_points.yaml 完全一样：
#   home_deg、cells{格: 6 关节角(度)}、zones{区域: [每个槽位 6 关节角]}。
# 用它可以在 Gazebo 里把示教回放模式整条链路跑通；真机上这份文件由现场示教得到（ros2 run mecharm_real teach ...）。
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mecharm_sort.planner import Geometry, Unreachable, plan_cell, plan_slot  # noqa: E402


def main(grid_path, out_path):
    cfg = yaml.safe_load(open(grid_path, encoding='utf-8'))
    geo = Geometry(cfg)
    out = {'home_deg': [0.0] * 6, 'cells': {}, 'zones': {}}
    for cid in sorted(geo.cells):
        try:
            out['cells'][cid] = [round(float(v), 2) for v in np.degrees(plan_cell(geo, cid)['pick'])]
        except Unreachable as e:
            print(f'网格 {cid} 不可达，不写入：{e}')
    for zid, z in geo.zones.items():
        out['zones'][zid] = []
        for slot in range(len(z['slots'])):
            out['zones'][zid].append([round(float(v), 2) for v in np.degrees(plan_slot(geo, zid, slot)['place'])])
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('# 示教点（仿真：由 make_taught_points.py 从逆解生成；真机：由现场示教写入）。单位：度。\n')
        f.write('# cells: 网格编号 → 取物关节角（夹爪张开、指垫夹住方块中部的高度）；zones: 区域 → 每个槽位的放置关节角。\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=200)
    print(f'已写入 {out_path}：{len(out["cells"])} 个取物点，{sum(len(v) for v in out["zones"].values())} 个放置点')


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, 'grid.yaml'),
         sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, 'taught_points_sim.yaml'))
