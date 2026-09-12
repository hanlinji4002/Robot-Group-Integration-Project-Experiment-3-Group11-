#!/usr/bin/env python3
# 【讲解】布局校验工具（离线，不需要 ROS）：调用与抓取服务器同一个规划器（mecharm_sort/planner.py），
# 对 grid.yaml 里"每个网格 × 每个分类区域 × 每个槽位"做完整规划，并检查服务器实际会走的每一段关节空间直线：
# 夹爪上 9 个关键点（指尖/指垫/基座两端）在网格+料盒区域上空时最低点不得低于 0.035（方块顶 0.025 + 1 cm 余量）。
# 用法：python3 check_layout.py [grid.yaml]    改了网格/料盒/相机位置先跑一遍，全部通过再上仿真。
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from mecharm_sort.kinematics import fk_points, gripper_points  # noqa: E402
from mecharm_sort.planner import Geometry, Unreachable, plan_pick_place, path_segments  # noqa: E402

TOOL = 0.063
HOME_POSE = np.radians([0, 0, 0, 0, 0, 0])   # 回零 = 全零（大臂竖直、小臂水平向前）
MIN_Z = 0.035


def main(path):
    cfg = yaml.safe_load(open(path, encoding='utf-8'))
    geo = Geometry(cfg)
    region = lambda p: -0.02 < p[0] < 0.26 and -0.20 < p[1] < 0.20
    bad, n_plans, worst = 0, 0, (9.0, None)
    print(f'== 网格 {sorted(geo.cells)} × 区域 {list(geo.zones)} × 槽位 全组合规划 + 路径最低点检查')
    for cid in sorted(geo.cells):
        for bid, b in geo.zones.items():
            for slot in range(len(b['slots'])):
                tag = f'格{cid}→{bid}[{slot}]'
                try:
                    wp, _ = plan_pick_place(geo, cid, bid, slot)
                except Unreachable as e:
                    print(f'  FAIL {tag:14s} {e}')
                    bad += 1   # grid_abnormal.yaml 里 6 号格故意放在不可达处，这里报 FAIL 是预期的
                    continue
                n_plans += 1
                cell_xy = np.array(geo.cells[cid]) - geo.base[:2]
                sl = b['slots'][slot]
                slot_xy = np.array([b['x'] + sl[0], b['y'] + sl[1]]) - geo.base[:2]
                for seg, qa, qb in path_segments(wp, HOME_POSE):
                    # 下降/抬升段本来就要贴到方块高度，只检查目标点 4 cm 以外（邻位方块处）的最低点
                    excl = None
                    if 'pick' in seg:
                        excl = cell_xy
                    elif 'place' in seg:
                        excl = slot_xy
                    for s_ in np.linspace(0, 1, 50):
                        q = qa + (qb - qa) * s_
                        for p in gripper_points(q, TOOL):
                            if excl is not None and np.linalg.norm(p[:2] - excl) < 0.04:
                                continue
                            if region(p) and p[2] < worst[0]:
                                worst = (p[2], f'{tag} 段 {seg[0]}→{seg[1]}')
                # 打印关节角便于人工检查构型是否合理（J1 应接近目标方位角）
                if slot == 0 and bid == list(geo.zones)[0]:
                    print(f'  ok   格{cid} pick J={np.round(np.degrees(wp["pick"]), 1).tolist()}')
                print(f'       {tag:14s} place J={np.round(np.degrees(wp["place"]), 1).tolist()}')
    print(f'  可达规划 {n_plans} 条；路径最低点 z = {worst[0]:.3f}（{worst[1]}），阈值 {MIN_Z}')
    if worst[0] < MIN_Z:
        bad += 1
    print('== 回零姿态各点（基座系）：', [tuple(np.round(p, 3)) for p in fk_points(HOME_POSE, TOOL)][-3:])
    print('结果：', '全部通过' if bad == 0 else f'{bad} 项不通过')
    return bad


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), 'grid.yaml')) else 0)
