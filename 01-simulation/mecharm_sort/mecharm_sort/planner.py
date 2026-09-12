#!/usr/bin/env python3
# 【讲解】固定点抓取规划器（纯计算，不依赖 ROS）：网格编号 → 取物侧路径点；区域编号 + 槽位 → 放置侧路径点。
# 抓取服务器和 config/check_layout.py 都调用这里，保证"离线校验"与"在线执行"用的是同一套规划。
# 取物侧与放置侧分开求解、分开缓存（6 格 + 9 个落点 = 15 组，启动时几十秒算完），任务里直接查表。
# 执行顺序：回零 → above_cell → pick → (above_cell) → transit → carry(只转基座到区域方位) → above_zone → place → (above_zone) → carry → back(只转基座回正前) → 回零。
# 任一点超工作半径 / 无逆解 / 超限 → 抛 Unreachable（服务器转成 failed_stage=plan，任务节点记"不可达"并跳过）。
import numpy as np

from mecharm_sort.kinematics import check_limits, solve_aligned, solve_ik


class Unreachable(Exception):
    pass


class Geometry:
    """从 grid.yaml 解析出的桌面几何（世界坐标）。"""

    def __init__(self, cfg):
        self.base = np.array(cfg['robot_base_xyz'], dtype=float)
        self.table_h = float(cfg['table_height'])
        self.obj = float(cfg['object_size'])
        self.cells = {int(k): (float(v['x']), float(v['y'])) for k, v in cfg['cells'].items()}
        self.zones = {k: {'x': float(v['x']), 'y': float(v['y']),
                          'slots': [list(map(float, s)) for s in v['slots']],
                          'jaw_axis': str(v.get('jaw_axis', 'y')),
                          'half': list(map(float, v.get('half', [0.07, 0.04])))}
                      for k, v in cfg['zones'].items()}

    @property
    def zc(self):
        """方块中心的世界高度。"""
        return self.table_h + self.obj / 2

    # 【讲解】某个点是否落在分类区域矩形内（放置成功判定）
    def in_zone(self, zone_id, xy, margin=0.0):
        z = self.zones[zone_id]
        return abs(xy[0] - z['x']) <= z['half'][0] + margin and abs(xy[1] - z['y']) <= z['half'][1] + margin


DEFAULT_PARAMS = dict(tool_tip_offset=0.063, safe_height=0.05, transit_height=0.08,
                      place_height=0.008, place_above=0.05, pick_tilt=0.26, place_tilt=0.26,
                      transit_tilt=0.26, jaw_axis='y', max_reach=0.32)


def _params(params):
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    return p


def _solve(name, tgt, tool, seed, tilt, axis, max_reach):
    reach = float(np.linalg.norm(tgt - np.array([0, 0, 0.138])))  # 肩部球心近似
    if reach > max_reach:
        raise Unreachable(f'路径点 {name} {np.round(tgt, 3).tolist()} 超出工作半径（{reach:.3f} m > {max_reach} m），目标不可达')
    q = solve_aligned(tgt, tool, seed=seed, tilt_tol=tilt, axis=axis)
    if q is None:  # 对齐后不可达就退回不对齐（区域落点在可达边缘）
        q = solve_ik(tgt, tool, seed=seed, tilt_tol=tilt)
    if q is None:
        raise Unreachable(f'路径点 {name} {np.round(tgt, 3).tolist()} 无逆解，目标不可达')
    ok, why = check_limits(q)
    if not ok:
        raise Unreachable(f'路径点 {name} {why}')
    return q


# 【讲解】取物侧：网格编号 → above_cell / pick / transit 三个路径点（工具竖直，虎口沿 jaw_axis）
def plan_cell(geo, grid_id, params=None):
    p = _params(params)
    if grid_id not in geo.cells:
        raise Unreachable(f'网格编号 {grid_id} 不存在')
    cx, cy = geo.cells[grid_id]
    cell = np.array([cx, cy, geo.zc]) - geo.base
    wp = {}
    wp['above_cell'] = _solve('above_cell', cell + [0, 0, p['safe_height']], p['tool_tip_offset'], None, p['pick_tilt'], p['jaw_axis'], p['max_reach'])
    wp['pick'] = _solve('pick', cell + [0, 0, 0.002], p['tool_tip_offset'], wp['above_cell'], p['pick_tilt'], p['jaw_axis'], p['max_reach'])
    wp['transit'] = _solve('transit', cell + [0, 0, p['transit_height']], p['tool_tip_offset'], wp['above_cell'], p['transit_tilt'], p['jaw_axis'], p['max_reach'])
    return wp


# 【讲解】放置侧：区域编号 + 槽位 → above_zone / place 两个路径点（工具竖直、虎口沿区域的 jaw_axis；夹着方块时不能倾斜，否则会滑出）
def plan_slot(geo, zone_id, slot, params=None):
    p = _params(params)
    if zone_id not in geo.zones:
        raise Unreachable(f'区域编号 {zone_id} 不存在')
    z = geo.zones[zone_id]
    dx, dy = z['slots'][slot % len(z['slots'])]
    pt = np.array([z['x'] + dx, z['y'] + dy, geo.zc]) - geo.base
    wp = {}
    wp['above_zone'] = _solve('above_zone', pt + [0, 0, p['place_above']], p['tool_tip_offset'], None, p['place_tilt'], z['jaw_axis'], p['max_reach'])
    wp['place'] = _solve('place', pt + [0, 0, p['place_height']], p['tool_tip_offset'], wp['above_zone'], p['place_tilt'], z['jaw_axis'], p['max_reach'])
    return wp


# 【讲解】拼成一次完整取放的路径点表；carry/back 只改基座 J1，让横移沿等高圆弧扫过，不会半路下沉擦到方块
def compose(cell_wp, slot_wp):
    wp = dict(cell_wp)
    wp.update(slot_wp)
    carry = wp['transit'].copy()
    carry[0] = wp['above_zone'][0]
    wp['carry'] = carry
    # 放完先抬回 carry（过渡高度、区域方位），再只转 J1 回正前（back），最后回零：全程在过渡高度以上横移，
    # 放置点即使教得很低（或空中放置模式下放置点=上方点）也不会贴着桌面扫回来
    back = carry.copy()
    back[0] = 0.0
    wp['back'] = back
    return wp


def plan_pick_place(geo, grid_id, zone_id, slot, params=None):
    return compose(plan_cell(geo, grid_id, params), plan_slot(geo, zone_id, slot, params)), geo.cells[grid_id]


# 服务器执行的关节空间路径段（用于离线最低点检查）
def path_segments(wp, home_q):
    names = ['home', 'above_cell', 'pick', 'above_cell', 'transit', 'carry', 'above_zone', 'place',
             'above_zone', 'carry', 'back', 'home']
    pts = [home_q if n == 'home' else wp[n] for n in names]
    return [((names[i], names[i + 1]), pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
