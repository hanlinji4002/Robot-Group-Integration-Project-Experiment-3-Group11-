#!/usr/bin/env python3
# 【讲解】mechArm 270 运动学模块（实验三）。从实验二的任务节点里抽出来单独成文件，
# 供抓取动作服务器（pick_place_server）和布局校验脚本（config/check_layout.py）共用。
# 内容：正运动学 fk / 指尖位置 tip_pos / 数值逆解 solve_ik / 虎口对齐 align_jaw / 限位表 JOINT_LIMITS。
# 讲解要点：链参数逐字段抄自官方 URDF；逆解是阻尼最小二乘牛顿迭代，收敛不了 = 目标不可达。
"""mechArm 270 正/逆运动学（纯 numpy，不依赖 ROS）。"""
import math

import numpy as np

ARM_JOINTS = [
    'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
    'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5',
]
GRIPPER_JOINT = 'gripper_controller'

# 关节限位，与 URDF 一致（rad）
JOINT_LIMITS = [
    (-2.792527, 2.792527), (-1.3089, 2.0943), (-3.0543, 1.1344),
    (-2.7052, 2.7052), (-2.0071, 2.0071), (-3.14, 3.14),
]


# 【讲解】欧拉角(roll,pitch,yaw) -> 3x3 旋转矩阵，URDF 里 origin 的 rpy 就是这个约定
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


# 【讲解】平移+旋转 -> 4x4 齐次变换矩阵，链式相乘就能算出末端位置
def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


# 各关节前置静态变换（parent→child origin），之后绕子坐标系 z 轴转 q
CHAIN = [
    make_T([0, 0, 0.1], [0, 0, 0]),
    make_T([0, 0, 0.038], [-1.5708, 0, 0]),
    make_T([0.0, -0.1, 0], [0, 0, 0]),
    make_T([0.108, -0.005, -0.001], [0, 1.5708, 0]),
    make_T([-0.001, 0, 0.0], [0, -1.5708, 0]),
    make_T([0.06, 0.0, -0.0], [0, 1.5708, 0]),
]
T_FLANGE = make_T([0, 0, 0.038], [1.579, 0, 0])  # link6 → gripper_base


def rz(q):
    T = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


# 【讲解】正运动学：给六个关节角，算出 link6 与夹爪基座在基座坐标系的位姿
def fk(q):
    """返回 (link6 位姿 4x4, gripper_base 位姿 4x4)，基座坐标系。"""
    T = np.eye(4)
    for i in range(6):
        T = T @ CHAIN[i] @ rz(q[i])
    return T, T @ T_FLANGE


# 【讲解】正运动学扩展版：顺带返回每个关节原点、法兰和指尖的位置，用于碰撞/遮挡检查
def fk_points(q, tool_offset):
    """返回 [基座, J1..J6 原点, 法兰, 指尖] 共 9 个点（基座坐标系）。"""
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for i in range(6):
        T = T @ CHAIN[i] @ rz(q[i])
        pts.append(T[:3, 3].copy())
    Tg = T @ T_FLANGE
    pts.append(Tg[:3, 3].copy())
    pts.append(Tg[:3, 3] + T[:3, 2] * tool_offset)
    return pts


# 官方夹爪的指垫相对法兰中心有 1 cm 侧向偏置（URDF 里指垫碰撞盒 z=-0.010、整套网格 oz=-0.012），
# 实验三仿真真值实测确认：不计这个偏置，指垫会偏离方块中心 1 cm。这里作为模块级默认值，抓取服务器可用参数覆盖
TOOL_LATERAL = -0.010


# 【讲解】指尖位置 = 夹爪基座原点沿接近轴(gripper y)前伸 tool_offset，再沿 gripper z 偏 TOOL_LATERAL（指垫中心）
def tip_pos(q, tool_offset, lateral=None):
    """虎口中心位置（基座系）与工具接近轴方向。"""
    lat = TOOL_LATERAL if lateral is None else lateral
    T6, Tg = fk(q)
    return Tg[:3, 3] + Tg[:3, 1] * tool_offset + Tg[:3, 2] * lat, Tg[:3, 1]


# 【讲解】数值逆解：多初值牛顿迭代，让指尖到目标点且工具轴接近竖直向下；都收敛不了就返回 None = 不可达。
# tilt_tol 是允许的姿态误差 |axis-down|（0.26≈15°，用于抓取；0.68≈40°，用于往料盒里放，放置不需要那么正）
def solve_ik(target, tool_offset, seed=None, iters=150, tilt_tol=0.26):
    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])
    seeds = [seed] if seed is not None else []
    yaw = math.atan2(target[1], target[0])
    seeds += [
        # 经验初值：前两组是实测抓取构型族（J2 20-40°、J3 -20-25°、J5 55-70°、J6≈方位角），后三组沿用实验二
        np.array([yaw, 0.35, 0.2, 0.0, 1.0, yaw]),
        np.array([yaw, 0.7, -0.3, 0.0, 1.15, yaw]),
        np.array([yaw, 0.8, -1.6, 0.0, -0.8, 0.0]),
        np.array([yaw, 0.4, -1.0, 0.0, -1.0, 0.0]),
        np.array([yaw, 1.2, -2.0, 0.0, -0.6, 0.0]),
    ]
    rng = np.random.default_rng(42)  # 固定种子保证可复现
    seeds += [lo + rng.random(6) * (hi - lo) for _ in range(12)]
    down = np.array([0, 0, -1.0])
    AXIS_W = 0.4
    # 【讲解】初值策略：先把"给定初值 + 3 个经验初值"都跑一遍，收敛的解里选离参考构型最近的（避免手臂翻到另一侧、
    # 肘部反折这类奇怪构型）；全都不收敛才试 12 个随机兜底初值，取第一个收敛的。迭代内步长极小时提前退出。
    q_ref = seed if seed is not None else np.array([yaw, 0.6, -1.2, 0.0, -0.8, 0.0])
    n_analytic = len(seeds) - 12

    def run(q):
        q = np.clip(np.asarray(q, dtype=float).copy(), lo, hi)
        for _ in range(iters):
            p, axis = tip_pos(q, tool_offset)
            r = np.concatenate([p - target, AXIS_W * (axis - down)])
            if np.linalg.norm(p - target) < 0.002 and np.linalg.norm(axis - down) < tilt_tol:
                return np.clip(q, lo, hi)
            J = np.zeros((6, 6))
            eps = 1e-5
            for i in range(6):
                dq = q.copy()
                dq[i] += eps
                p2, a2 = tip_pos(dq, tool_offset)
                J[:, i] = np.concatenate([(p2 - p), AXIS_W * (a2 - axis)]) / eps
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ r)
            q_new = np.clip(q - np.clip(step, -0.3, 0.3), lo, hi)
            if np.linalg.norm(q_new - q) < 1e-7:
                return None
            q = q_new
        return None

    # 硬性过滤：基座转角必须朝着目标方位（|J1-yaw| ≤ 90°），"绕到另一侧反手够"的解一律不要——
    # 那种构型手臂会横扫整个桌面，宁可判不可达
    def facing(sol):
        d = (sol[0] - yaw + math.pi) % (2 * math.pi) - math.pi
        return abs(d) <= math.pi / 2

    solutions = [sol for sol in (run(q) for q in seeds[:n_analytic]) if sol is not None and facing(sol)]
    if solutions:
        w = np.array([3.0, 1, 1, 0.5, 1, 0.2])
        return min(solutions, key=lambda s_: np.linalg.norm((s_ - q_ref) * w))
    for q in seeds[n_analytic:]:
        sol = run(q)
        if sol is not None and facing(sol):
            return sol
    return None


# 【讲解】转 J6 让虎口开合方向对齐指定的世界轴（'x' 或 'y'），斜着夹会把方块挤出去。
# 网格里相邻方块沿 y 间距 7cm、沿 x 间距 4.5cm，手指沿 y 张开时离邻格方块最远，所以默认对齐 y 轴
def align_jaw(q, axis='y'):
    q = q.copy()
    target_phi = 0.0 if axis == 'x' else math.pi / 2
    for _ in range(6):
        _, Tg = fk(q)
        jaw = Tg[:3, 0]
        phi = math.atan2(jaw[1], jaw[0])
        # 与目标轴的夹角，模 π（虎口是对称的，正反向等价）
        delta = phi - target_phi
        delta = (delta + math.pi / 2) % math.pi - math.pi / 2
        if abs(delta) < 0.01:
            break
        q2 = q.copy()
        q2[5] += 0.01
        _, Tg2 = fk(q2)
        jaw2 = Tg2[:3, 0]
        dphi = (math.atan2(jaw2[1], jaw2[0]) - phi)
        dphi = ((dphi + math.pi) % (2 * math.pi) - math.pi) / 0.01
        if abs(dphi) < 1e-3:
            break
        q[5] = np.clip(q[5] - delta / dphi, JOINT_LIMITS[5][0], JOINT_LIMITS[5][1])
    # 虎口左右对称，J6 转 180° 等价：把 J6 归一到 (-90°, 90°]，避免逆解给出 ±180° 附近的解让手腕大幅翻转
    while q[5] > math.pi / 2:
        q[5] -= math.pi
    while q[5] <= -math.pi / 2:
        q[5] += math.pi
    return q


# 【讲解】夹爪上几个关键点（基座系）：指尖、两个指垫外沿底部、夹爪基座两端。用于路径最低点检查——
# 手腕翻转时指垫可能比"指尖"更低，只看指尖会漏掉擦到方块顶的情况
def gripper_points(q, tool_offset):
    T6, Tg = fk(q)
    tip = Tg[:3, 3] + T6[:3, 2] * tool_offset
    jaw = Tg[:3, 0]
    appr = T6[:3, 2]
    pts = [tip]
    for sgn in (1, -1):
        pad = tip + jaw * sgn * 0.0325
        pts.append(pad + appr * 0.0075)   # 指垫底部（沿接近轴再伸 7.5mm）
        pts.append(pad - appr * 0.03)     # 指根
        pts.append(Tg[:3, 3] + jaw * sgn * 0.0325)  # 夹爪基座两端
    return pts


# 【讲解】限位检查：返回 (是否合法, 说明文字)
def check_limits(q):
    for j, (lo, hi) in enumerate(JOINT_LIMITS):
        if not (lo - 1e-6 <= q[j] <= hi + 1e-6):
            return False, f'{ARM_JOINTS[j]} 目标 {math.degrees(q[j]):.1f}° 超限 [{math.degrees(lo):.0f},{math.degrees(hi):.0f}]°'
    return True, ''


# 【讲解】"逆解 + 虎口对齐"打包：对齐会转 J6，而指垫有侧向偏置，转 J6 会把虎口中心带偏几毫米，
# 所以对齐后用对齐结果做初值再解一次，迭代到位置误差 < 2 mm。返回 None = 不可达
def solve_aligned(target, tool_offset, seed=None, tilt_tol=0.26, axis='y', rounds=3):
    q = solve_ik(target, tool_offset, seed=seed, tilt_tol=tilt_tol)
    if q is None:
        return None
    for _ in range(rounds):
        q = align_jaw(q, axis)
        p, _ = tip_pos(q, tool_offset)
        if np.linalg.norm(p - target) < 0.002:
            return q
        q2 = solve_ik(target, tool_offset, seed=q, tilt_tol=tilt_tol)
        if q2 is None:
            return None
        q = q2
    return align_jaw(q, axis)
