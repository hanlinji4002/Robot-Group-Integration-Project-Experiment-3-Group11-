#!/usr/bin/env python3
# 【讲解】示教工具，跑在 Jetson（纯标准库，不需要 rclpy）。
# 流程：张开夹爪 → 变软（release_servo J1–J5）→ 人手把臂摆到位 → 回车记录关节角 → 恢复力矩。
# apply 时把 at_a/at_b 关节角写进 real.yaml（use_taught_joints: true，任务节点直接回放这些角度），
# 同时用与任务节点相同的正运动学算出对应坐标写进 point_a/point_b 供参考。
# 讲解要点：示教点存在臂内 taught_points.json；恢复力矩在这台固件上要约 10 s，所以有重试。
"""示教工具（Jetson 侧，纯标准库，不需要 rclpy）。

通过臂内 arm_server.py 完成「变软 -> 手摆 -> 记录 -> 恢复力矩」，
再用与任务节点相同的正运动学把示教关节角换算成 point_a / point_b 写进 real.yaml。
这样任务节点（ros_node.py）不用改，只是参数变了——正是验收要求的做法。

用法：
  ros2 run mecharm_real teach a            # 示教取物点 A（夹爪张开套住目标物）
  ros2 run mecharm_real teach b            # 示教放置点 B
  ros2 run mecharm_real teach show         # 看已示教的点及其换算结果
  ros2 run mecharm_real teach apply        # 把 A/B 换算后写进 real.yaml
  可加 --host 10.42.0.89 --port 9001 --yaml /path/real.yaml --json taught_points.json（离线换算）
"""
import argparse
import json
import math
import os
import re
import sys

# 终端不是 UTF-8（如 nohup / systemd 下 LANG=C）时中文日志也不能把进程打死
def _utf8_stdout():
    import io, sys
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:  # Python < 3.7
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
_utf8_stdout()

from mecharm_real.arm_client import ArmClient, ArmError
from mecharm_real.kinematics import tip_pos, tilt_deg, reach, MAX_REACH, deg_to_rad

# 任务节点 plan_waypoints 在 point_a/point_b 上叠加的高度（at_a = A + 0.002，at_b = B + 0.008），
# 示教教的是 at_a / at_b 本身，换算回 point 时要减掉
AT_A_LIFT, AT_B_LIFT = 0.002, 0.008
MAX_TILT = 15.0   # 任务节点 IK 允许的工具轴倾角


# 【讲解】通过 ament 找到已安装的 real.yaml 路径（symlink-install 下就是源码里那份）
def default_yaml():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('mecharm_real'), 'config', 'real.yaml')
    except Exception:
        return None


def read_yaml_scalar(text, key, default):
    m = re.search(r'^\s*%s:\s*([-\d.]+)' % re.escape(key), text, re.M)
    return float(m.group(1)) if m else default


# 【讲解】示教关节角 → 指尖坐标（减去任务节点在 point 上叠加的抬升量），并算工具轴倾角与工作半径
def convert(name, angles_deg, tool_offset, lift):
    q = deg_to_rad(angles_deg)
    p, axis = tip_pos(q, tool_offset)
    point = [round(p[0], 4), round(p[1], 4), round(p[2] - lift, 4)]
    return point, tilt_deg(axis), reach(p)


# 【讲解】打印一个示教点的换算结果与告警（告警只影响逆解模式）
def report(name, angles, tool_offset, lift):
    point, tilt, r = convert(name, angles, tool_offset, lift)
    flags = []
    if tilt > MAX_TILT:
        flags.append('工具轴倾角 %.1f° > %d°，任务节点逆解可能不认' % (tilt, MAX_TILT))
    if r > MAX_REACH:
        flags.append('超出工作半径 %.3f > %.2f' % (r, MAX_REACH))
    print('  %-5s 关节角 %s' % (name, angles))
    print('        -> point %s  倾角 %.1f°  半径 %.3f m  %s' % (point, tilt, r, '⚠ ' + '；'.join(flags) if flags else '✔'))
    return point, flags


# 【讲解】交互式示教一个点：开爪 → 变软 → 等人摆位 → 记录 → 恢复力矩（失败重试 3 次）
def teach_one(cli, name, tip):
    print('[0] 张开夹爪，先套住目标物再教')
    cli.gripper(0)
    if input('[1] 即将变软：J1-J5 失去力矩会往下坠，先用手扶住机械臂！ [回车继续 / q 退出] ').strip().lower() == 'q':
        return None
    cli.soft()
    print('已变软，可以手动摆位')
    if input('[2] 把臂摆到 %s：%s  [回车记录 / q 退出] ' % (name, tip)).strip().lower() == 'q':
        cli.hold()
        return None
    resp = cli.record(name)
    print('  已记录 %s = %s' % (name, resp['angles']))
    input('[3] 即将恢复力矩（臂会在当前位置锁住，可能要十几秒，期间继续扶着） [回车继续] ')
    for attempt in (1, 2, 3):
        try:
            cli.hold()
            print('  力矩已恢复，可以松手')
            break
        except ArmError as e:
            print('  恢复力矩第 %d 次未收到回执（%s），臂可能仍是软的，继续扶住，重试...' % (attempt, e))
    else:
        print('  !!! 三次都没回执。先别松手，在臂内运行: python3 /home/er/armtest2.py hold')
    return resp['angles']


# 【讲解】子命令 a / b 示教，show 查看，apply 写入 real.yaml
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('what', choices=['a', 'b', 'show', 'apply'])
    ap.add_argument('--host', default='10.42.0.89')
    ap.add_argument('--port', type=int, default=9001)
    ap.add_argument('--yaml', default=default_yaml())
    ap.add_argument('--json', default=None, help='离线：直接读 taught_points.json，不连臂')
    args = ap.parse_args()

    if not args.yaml or not os.path.exists(args.yaml):
        sys.exit('找不到 real.yaml，请用 --yaml 指定')
    ytext = open(args.yaml, encoding='utf-8').read()
    tool_offset = read_yaml_scalar(ytext, 'tool_tip_offset', 0.063)

    if args.json:
        pts = json.load(open(args.json))
        cli = None
    else:
        cli = ArmClient(args.host, args.port)
        try:
            cli.ping()
        except ArmError as e:
            sys.exit('连不上臂内服务 %s:%d（%s）。先在臂内跑 python3 /home/er/arm_server.py' % (args.host, args.port, e))
        pts = None

    if args.what in ('a', 'b'):
        if cli is None:
            sys.exit('示教需要连臂，不能用 --json')
        name, tip = ('at_a', '取物点 A（夹爪张开套住目标物）') if args.what == 'a' else ('at_b', '放置点 B（放下物体的位置）')
        angles = teach_one(cli, name, tip)
        if angles is None:
            print('已取消'); return
        print('\n换算结果（tool_tip_offset=%.3f）:' % tool_offset)
        report(name, angles, tool_offset, AT_A_LIFT if args.what == 'a' else AT_B_LIFT)
        print('\n>>> 下一步: %s' % ('ros2 run mecharm_real teach b' if args.what == 'a' else 'ros2 run mecharm_real teach apply'))
        return

    pts = pts if pts is not None else cli.points()
    missing = [k for k in ('at_a', 'at_b') if k not in pts]
    if missing:
        sys.exit('示教点缺少 %s，先跑 teach a / teach b' % missing)
    print('已示教的点（tool_tip_offset=%.3f）:' % tool_offset)
    pa, fa = report('at_a', pts['at_a'], tool_offset, AT_A_LIFT)
    pb, fb = report('at_b', pts['at_b'], tool_offset, AT_B_LIFT)
    if args.what == 'show':
        return
    if fa or fb:
        print('（以上告警只影响逆解模式；默认的示教回放模式直接回放关节角，不受影响）')
    def set_line(text, key, value):
        pat = r'^(\s*%s:\s*).*$' % re.escape(key)
        if not re.search(pat, text, re.M):
            sys.exit('real.yaml 里没找到 %s 行，未修改' % key)
        return re.sub(pat, lambda m: m.group(1) + value, text, count=1, flags=re.M)
    new = set_line(ytext, 'point_a', str(pa))
    new = set_line(new, 'point_b', str(pb))
    new = set_line(new, 'use_taught_joints', 'true')
    new = set_line(new, 'at_a_joints_deg', str([round(float(v), 2) for v in pts['at_a']]))
    new = set_line(new, 'at_b_joints_deg', str([round(float(v), 2) for v in pts['at_b']]))
    open(args.yaml, 'w', encoding='utf-8').write(new)
    print('\n已写入 %s（示教回放模式已打开）' % args.yaml)
    print('  at_a_joints_deg: %s\n  at_b_joints_deg: %s' % (pts['at_a'], pts['at_b']))
    print('  point_a: %s\n  point_b: %s  （仅供参考/逆解模式用）' % (pa, pb))
    print('>>> 下一步: ros2 launch mecharm_real real.launch.py cycles:=1')


if __name__ == '__main__':
    main()
