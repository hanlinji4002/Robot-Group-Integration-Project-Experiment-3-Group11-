# 【讲解】纯 Python 版运动学，与仿真任务节点 ros_node.py 的 CHAIN / T_FLANGE 完全一致（数值抄自官方 URDF）。
# 不依赖 numpy / rclpy，所以 teach.py 在哪台机器都能算。
# 另外放了几个小换算：角度<->弧度（含方向/偏置修正）、"角度差/时长"->速度百分比、夹爪弧度->开/合。
"""纯 Python 运动学（不依赖 numpy / rclpy，Pi、Jetson、Mac 都能跑）。

链参数与 01-simulation/mecharm_grasp/ros_node.py 的 CHAIN / T_FLANGE 完全一致，
这样示教点经这里的正运动学换算出的 point_a/point_b，任务节点的逆解能重新解回同一姿势。
"""
import math

ARM_JOINTS = [
    'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
    'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5',
]
GRIPPER_JOINT = 'gripper_controller'

# 关节限位（rad），与 URDF / 任务节点一致
JOINT_LIMITS = [
    (-2.792527, 2.792527), (-1.3089, 2.0943), (-3.0543, 1.1344),
    (-2.7052, 2.7052), (-2.0071, 2.0071), (-3.14, 3.14),
]
MAX_JOINT_DEG_S = 120.0      # 官方规格最大关节速度
SHOULDER = (0.0, 0.0, 0.138)  # 肩部球心近似，任务节点用它做工作半径检查
MAX_REACH = 0.30


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def make_T(xyz, rpy):
    R = rpy_mat(*rpy)
    return [R[0] + [xyz[0]], R[1] + [xyz[1]], R[2] + [xyz[2]], [0, 0, 0, 1]]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rz(q):
    c, s = math.cos(q), math.sin(q)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


CHAIN = [
    make_T([0, 0, 0.1], [0, 0, 0]),
    make_T([0, 0, 0.038], [-1.5708, 0, 0]),
    make_T([0.0, -0.1, 0], [0, 0, 0]),
    make_T([0.108, -0.005, -0.001], [0, 1.5708, 0]),
    make_T([-0.001, 0, 0.0], [0, -1.5708, 0]),
    make_T([0.06, 0.0, -0.0], [0, 1.5708, 0]),
]
T_FLANGE = make_T([0, 0, 0.038], [1.579, 0, 0])  # link6 -> gripper_base


# 【讲解】正运动学：六个关节角 → link6 与夹爪基座的 4x4 位姿
def fk(q_rad):
    """返回 (link6 位姿 4x4, gripper_base 位姿 4x4)，基座坐标系。"""
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for i in range(6):
        T = matmul(matmul(T, CHAIN[i]), rz(q_rad[i]))
    return T, matmul(T, T_FLANGE)


# 【讲解】指尖位置 = 夹爪基座沿工具轴前伸 tool_offset；同时返回工具轴方向
def tip_pos(q_rad, tool_offset):
    """指尖位置与工具接近轴（基座系）：gripper_base 原点沿 link6 z 轴前伸 tool_offset。"""
    T6, Tg = fk(q_rad)
    axis = [T6[0][2], T6[1][2], T6[2][2]]
    p = [Tg[i][3] + axis[i] * tool_offset for i in range(3)]
    return p, axis


# 【讲解】工具轴与竖直向下的夹角，逆解模式要求约 15° 以内
def tilt_deg(axis):
    """工具轴与竖直向下的夹角（度）。任务节点的 IK 要求约 <=15°。"""
    d = max(-1.0, min(1.0, -axis[2]))
    return math.degrees(math.acos(d))


def reach(p):
    return math.sqrt(sum((p[i] - SHOULDER[i]) ** 2 for i in range(3)))


def deg_to_rad(degs, signs=None, offsets=None):
    signs = signs or [1.0] * 6
    offsets = offsets or [0.0] * 6
    return [math.radians((d - o) / s) for d, s, o in zip(degs, signs, offsets)]


def rad_to_deg(rads, signs=None, offsets=None):
    signs = signs or [1.0] * 6
    offsets = offsets or [0.0] * 6
    return [math.degrees(v) * s + o for v, s, o in zip(rads, signs, offsets)]


# 【讲解】把"要转多少度 / 用几秒"换成 pymycobot 的速度百分比，夹在 [5, max_speed]
def speed_percent(delta_deg, duration_s, max_speed, min_speed=5):
    """把「多少度 / 多少秒」换成 pymycobot 的速度百分比，夹在 [min_speed, max_speed]。"""
    duration_s = max(0.5, float(duration_s))
    pct = delta_deg / duration_s / MAX_JOINT_DEG_S * 100.0
    return int(max(min_speed, min(int(max_speed), round(pct))))


# 【讲解】任务节点发的夹爪弧度离 open 近就是开(0)，离 close 近就是合(1)
def gripper_state(rad, open_rad, close_rad):
    """夹爪关节角 -> 0 开 / 1 合：离哪个标定值近就取哪个。"""
    return 0 if abs(rad - open_rad) <= abs(rad - close_rad) else 1
