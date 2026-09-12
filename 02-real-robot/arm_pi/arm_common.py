#!/usr/bin/env python3
# 【讲解】臂内公共库，所有直连脚本和 arm_server 都用它。基于 pymycobot 3.6.3 的 MechArm270 类。
# 两条铁规矩：① 必须用 MechArm270 类（老 MyCobot 类的运动帧会被当前固件丢弃）② 指令返回 -1 不代表失败，成败一律以读回角度为准。
# 内容：connect 连串口；read6 读六关节角（容错）；LIMIT 自定限位与 send_angles_direct 直发角度（绕过库内置的保守限位表）；
# goto 同步移动并报残差；soft/hold 松/锁力矩；gripper 开合；teach_point 示教存点；load_points/lifted 读点与抬升。
# 真机小工具公共部分（pymycobot 3.6.3 + MechArm270 类；返回码 -1 不代表失败，一律以读回角度为准）
import time, sys
from pymycobot import MechArm270

# 【讲解】打开串口 /dev/ttyAMA0，波特率 1000000
def connect():
    mc = MechArm270("/dev/ttyAMA0", 1000000)
    time.sleep(0.8)
    return mc

# 【讲解】读六关节角，最多试 5 次；固件偶尔返回整数而非列表，当作没读到
def read6(fn, tries=5):
    for _ in range(tries):
        v = fn()
        # 固件没回数据时库会返回 -1/0 之类的整数，不是列表；当作没读到，继续重试
        if isinstance(v, (list, tuple)) and len(v) == 6:
            return list(v)
        time.sleep(0.4)
    return None

# 【讲解】打印角度与坐标
def show(mc, tag):
    print("%s 角度: %s" % (tag, read6(mc.get_angles)))
    print("%s 坐标: %s" % (tag, read6(mc.get_coords)))

# 按这台机械臂的实际活动范围自定的限位。
# pymycobot 库内置的 MechArm270 表把 J2 限在 ±90，但手摆到桌面取物时 J2 实际到 100 多度，
# 库的表是函数内局部变量改不了，所以这里自己发指令、自己查限位。
LIMIT_MIN = [-165, -90, -180, -165, -115, -175]
LIMIT_MAX = [165, 135, 70, 165, 115, 175]

# 【讲解】按 pymycobot 协议直接发角度帧，用上面自定的限位（库内置表把 J2 限在 ±90，取物时不够）
def send_angles_direct(mc, angles, speed):
    """按 pymycobot send_angles 的协议格式直接发角度，用上面自定的限位而不是库内置的表"""
    from pymycobot.common import ProtocolCode
    ints = [mc._angle2int(a) for a in angles]
    return mc._mesg(ProtocolCode.SEND_ANGLES, ints, speed, has_reply=True)

# 【讲解】先查限位，再发角度，轮询读回直到残差 < tol 或超时，返回到位角度与各关节残差
def goto(mc, target, speed, timeout=20, tol=1.5):
    """同步移动到 target（6 角），返回到位后的角度与最大残差"""
    for k, (v, lo, hi) in enumerate(zip(target, LIMIT_MIN, LIMIT_MAX)):
        if not lo <= v <= hi:
            mc.stop()
            sys.exit("!!! 目标 J%d=%.2f 超出限位 %d~%d，未发送" % (k + 1, v, lo, hi))
    print("-> 目标:", target, "速度:", speed)
    send_angles_direct(mc, target, speed)
    t0 = time.time()
    while time.time() - t0 < timeout:
        a = read6(mc.get_angles)
        if a and max(abs(x - t) for x, t in zip(a, target)) < tol:
            break
        time.sleep(0.2)
    time.sleep(0.5)
    a = read6(mc.get_angles)
    err = [round(abs(x - t), 2) for x, t in zip(a, target)] if a else None
    print("到位角度:", a, "(用时 %.1fs)" % (time.time() - t0))
    print("各关节残差:", err, "最大:", max(err) if err else None)
    return a, err

# ---------- 示教 / 抓取共用 ----------
import json, os, sys
POINTS_FILE = "/home/er/taught_points.json"

# 【讲解】J1–J5 松力矩可手动摆位，J6/夹爪保持刚性
def soft(mc):
    """J1-J5 变软可示教，J6/夹爪保持刚性"""
    for j in range(1, 6):
        mc.release_servo(j)
        time.sleep(0.1)

# 【讲解】恢复全部力矩（这台固件约 10 s）
def hold(mc):
    """恢复全部力矩"""
    for j in range(1, 7):
        mc.focus_servo(j)
        time.sleep(0.1)
    time.sleep(0.3)

# 【讲解】夹爪开(0)/合(1)，等 2 s 后读回开合值打印
def gripper(mc, state, name=""):
    """state: 0 开, 1 合"""
    r = mc.set_gripper_state(state, 50)
    time.sleep(2)
    try:
        v = mc.get_gripper_value()
    except Exception:
        v = None
    print("  夹爪%s 返回:%s 读值:%s" % (name, r, v))

# 【讲解】示教一个点：开爪 → 变软 → 人摆位 → 记录到 taught_points.json → 恢复力矩
def teach_point(name, tip):
    """变软 -> 手摆到 tip 描述的位置 -> 记录为 name -> 恢复力矩"""
    mc = connect()
    show(mc, "起始")
    gripper(mc, 0, "开")   # 张开夹爪再教，教出来的点才是套着物体的姿势
    if input("[1] 即将变软：J1-J5 会失去力矩往下坠，先用手扶住机械臂！  [回车继续 / q 退出] ").strip().lower() == "q":
        sys.exit(0)
    soft(mc)
    print("已变软，可以手动摆位")
    if input("[2] 把臂摆到 %s：%s  [回车记录 / q 退出] " % (name, tip)).strip().lower() == "q":
        hold(mc)
        sys.exit(0)
    a = read6(mc.get_angles)
    if not a:
        hold(mc)
        sys.exit("读不到角度，已恢复力矩并退出")
    pts = json.load(open(POINTS_FILE)) if os.path.exists(POINTS_FILE) else {}
    pts[name] = a
    json.dump(pts, open(POINTS_FILE, "w"), indent=1)
    print("  已记录 %s = %s" % (name, a))
    input("[3] 即将恢复力矩（臂会在当前位置锁住）  [回车继续] ")
    hold(mc)
    show(mc, "恢复力矩后")

# 【讲解】读示教点文件，缺哪个点就提示先去示教
def load_points(*names):
    """读 taught_points.json，缺一个就退出"""
    if not os.path.exists(POINTS_FILE):
        sys.exit("没有 %s，先跑 teach_a.py / teach_b.py 示教" % POINTS_FILE)
    pts = json.load(open(POINTS_FILE))
    missing = [k for k in names if k not in pts]
    if missing:
        sys.exit("taught_points.json 缺少 %s，先跑 teach_a.py / teach_b.py 示教" % missing)
    return pts

# 【讲解】在某点基础上把 J2 往回收 deg 度 = 垂直抬起（J2 正 = 前倾）
def lifted(pt, deg):
    """在 pt 基础上把 J2 往回收 deg 度，即抬起来（J2 正=前倾）"""
    up = list(pt)
    up[1] -= deg
    return up
