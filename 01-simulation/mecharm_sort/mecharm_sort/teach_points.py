#!/usr/bin/env python3
# 【讲解】实验三示教工具（Jetson 上跑，纯标准库 + PyYAML，不需要 rclpy）。
# 通过实验二的臂内 arm_server（mecharm_real.arm_client）完成：张爪 → 变软(J1-J5 松力矩) → 人手摆到位 → 回车读角度 → 恢复力矩，
# 把关节角写进 taught_points.yaml：cells 里 6 个取物点、zones 里每个区域若干放置点。抓取服务器示教回放模式直接读这个文件。
# 用法（先 source 工作区）：
#   ros2 run mecharm_sort teach_points cell 1              # 示教 1 号格取物点（方块放在格中心，夹爪张开套住它）
#   ros2 run mecharm_sort teach_points zone left 0         # 示教左侧区第 1 个放置点（方块底离桌面约 1 cm）
#   ros2 run mecharm_sort teach_points all                 # 按顺序把 6 格取物点 + 左/前/右各 1 个空中放置点教一遍（共 9 个）
#   ros2 run mecharm_sort teach_points show                # 看已教了哪些、还缺哪些
#   ros2 run mecharm_sort teach_points goto cell 1         # 低速回放到某个示教点核对（先确认周围没人）
#   ros2 run mecharm_sort teach_points hold                # 示教中断、臂还软着时恢复力矩（扶着臂再执行）
#   示教文件默认在 ~/Desktop/exp3_taught_points.yaml（工作区外面，同步代码不会覆盖）；可加 --host --port --file 改
"""示教工具：读臂当前关节角写入 taught_points.yaml。"""
import argparse
import os
import sys

import yaml


def _utf8_stdout():
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except AttributeError:
        pass


_utf8_stdout()

ZONES_DEFAULT = {'left': 1, 'front': 1, 'right': 1}   # all 模式每区教几个放置点：每种颜色 1 个，空中松爪扔下，人随手拿走
# 机械臂限位（度）：取实验二 arm_common 的真机限位与 URDF 限位中较严的一档。示教读数超出这个范围的点回放时会被拒绝
LIMIT_MIN = [-160, -75, -175, -155, -115, -175]
LIMIT_MAX = [160, 120, 65, 155, 115, 175]


def limit_problems(angles):
    bad = []
    for j, (v, lo, hi) in enumerate(zip(angles, LIMIT_MIN, LIMIT_MAX)):
        if not (lo <= v <= hi):
            bad.append(f'J{j + 1}={v:.1f}° 超出 [{lo},{hi}]')
    return bad


def default_file():
    # 示教点放在工作区外面（~/Desktop/exp3_taught_points.yaml），同步/重编译代码都不会把它覆盖掉
    return os.path.expanduser('~/Desktop/exp3_taught_points.yaml')


def load(path):
    data = {'home_deg': [0.0] * 6, 'cells': {}, 'zones': {}}
    if os.path.exists(path):
        data.update(yaml.safe_load(open(path, encoding='utf-8')) or {})
    data['cells'] = {int(k): (v or []) for k, v in (data.get('cells') or {}).items()}
    data['zones'] = {k: [list(p) for p in (v or [])] for k, v in (data.get('zones') or {}).items()}
    return data


def save(path, data):
    out = {'home_deg': data['home_deg'],
           'cells': {int(k): [round(float(x), 2) for x in v] for k, v in sorted(data['cells'].items())},
           'zones': {k: [[round(float(x), 2) for x in p] for p in v] for k, v in data['zones'].items()}}
    with open(path, 'w', encoding='utf-8') as f:
        f.write('# 示教点（真机）。单位：度。由 teach_points 工具写入；cells: 网格编号 → 取物关节角；zones: 区域 → 每个槽位的放置关节角。\n')
        f.write('# 空列表 [] = 还没示教，该格/该区域会被当作不可达跳过。\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=200)
    print(f'已写入 {path}')


def show(data):
    print('取物点：')
    for cid in range(1, 7):
        v = data['cells'].get(cid)
        print(f'  格 {cid}: {v if v else "（未示教）"}')
    print('放置点（每区 1 个，空中松爪）：')
    for z in ('left', 'front', 'right'):
        pts = data['zones'].get(z, [])
        good = [p for p in pts if len(p) == 6]
        print(f'  {z}: {len(good)} 个 ' + ('；'.join(str(p) for p in good) if good else '（未示教）'))
    missing = [c for c in range(1, 7) if not data['cells'].get(c)]
    missing += [z for z in ('left', 'front', 'right') if not [p for p in data['zones'].get(z, []) if len(p) == 6]]
    print('还缺：', missing if missing else '无，可以正式跑')


def teach_one(cli, tip):
    from mecharm_real.arm_client import ArmError
    print('[0] 张开夹爪')
    cli.gripper(0)
    if input('[1] 即将变软：J1-J5 失去力矩会往下坠，先用手扶住机械臂！ [回车继续 / q 退出] ').strip().lower() == 'q':
        return None
    cli.soft()
    print('    已变软，可以手动摆位')
    if input(f'[2] {tip}  [回车记录 / q 退出] ').strip().lower() == 'q':
        cli.hold()
        return None
    while True:
        resp = cli.get_angles()
        raw = resp.get('angles') if isinstance(resp, dict) else resp   # 臂内服务返回整包 JSON {ok, angles:[...]}
        angles = [float(a) for a in raw]
        print(f'    读到关节角 {angles}')
        bad = limit_problems(angles)
        if not bad:
            break
        print('    !!! 超出限位，回放时会被拒绝：' + '；'.join(bad))
        print('    常见原因：手腕翻转了（J4 接近 ±180、J5 超过 115）。把小臂转回来（J4 回到 0 附近），让 J5 在 ±115° 内朝下，再记录。')
        if input('    调整后回车重新读取 / q 放弃这个点 ').strip().lower() == 'q':
            cli.hold()
            return None
    input('[3] 即将恢复力矩（臂会在当前位置锁住，可能要十几秒，期间继续扶着） [回车继续] ')
    for attempt in (1, 2, 3):
        try:
            cli.hold()
            print('    力矩已恢复，可以松手')
            break
        except ArmError as e:
            print(f'    恢复力矩第 {attempt} 次未收到回执（{e}），臂可能仍是软的，继续扶住，重试...')
    else:
        print('    !!! 三次都没回执。先别松手，在臂内运行: python3 /home/er/armtest2.py hold')
    return angles


def main():
    ap = argparse.ArgumentParser(description='实验三示教工具')
    ap.add_argument('cmd', choices=['cell', 'zone', 'all', 'show', 'goto', 'hold', 'soft', 'angles'])
    ap.add_argument('args', nargs='*')
    ap.add_argument('--host', default='10.42.0.89')
    ap.add_argument('--port', type=int, default=9001)
    ap.add_argument('--file', default=default_file())
    ap.add_argument('--speed', type=int, default=15)
    ap.add_argument('--presim', action='store_true',
                    help='示教前先低速走到仿真逆解算出的该点上方姿态再变软（注意：真机 J5 方向与仿真不一致时会把手腕带歪，一般不用）')
    ap.add_argument('--no-verify', action='store_true', help='记录后不做回放验证')
    a = ap.parse_args()
    data = load(a.file)
    print(f'示教文件：{a.file}')
    if a.cmd == 'show':
        show(data)
        return
    from mecharm_real.arm_client import ArmClient
    cli = ArmClient(a.host, a.port, timeout=30.0)
    print('连接臂内服务：', cli.ping())
    if a.cmd == 'hold':        # 示教中断后臂还是软的：恢复力矩（扶着臂再执行）
        print(cli.hold())
        return
    if a.cmd == 'soft':
        input('即将松力矩 J1-J5，先扶住臂 [回车]')
        print(cli.soft())
        return
    if a.cmd == 'angles':
        print(cli.get_angles())
        return

    def presim(kind, key, slot=0):
        if not a.presim:
            return
        try:
            from ament_index_python.packages import get_package_share_directory
            sim = yaml.safe_load(open(os.path.join(get_package_share_directory('mecharm_sort'), 'config', 'taught_points_sim.yaml'), encoding='utf-8'))
            pose = list(sim['cells'][key]) if kind == 'cell' else list(sim['zones'][key][slot])
        except Exception as e:  # noqa: BLE001
            print(f'    读不到仿真示教点（{e}），跳过预摆位')
            return
        pose[1] -= 25.0
        input(f'    [预摆位] 低速({a.speed}%)走到仿真姿态上方 {[round(x, 1) for x in pose]}，确认周围无人 [回车]')
        print('   ', cli.goto(pose, a.speed))

    # 【讲解】记录后立刻回放验证：抬 25° 再回到该点，看机械臂是否真的走得到（有些手摆出来的姿态固件会拒绝执行，
    # 表现为"到位残差几十度"）。走不到就提示重教
    def verify(ang, label):
        if a.no_verify:
            return True
        up = list(ang)
        up[1] -= 25.0
        input(f'    [验证] 低速({a.speed}%)回放：先到上方点再回到 {label}，确认周围无人 [回车]')
        try:
            cli.goto(up, a.speed)
            r = cli.goto(ang, a.speed)
            err = r.get('err') or r.get('residual') or []
            worst = max([abs(float(e)) for e in err], default=0.0)
            if worst > 3.0:
                print(f'    !!! 回放到位残差 {worst:.1f}° > 3°，机械臂走不到这个姿态，请重教（换个更自然的手腕姿态）')
                return False
            print(f'    回放到位，残差 {worst:.1f}°')
            cli.goto(up, a.speed)
            return True
        except Exception as e:  # noqa: BLE001
            print(f'    !!! 回放失败：{e}。请重教这个点')
            return False

    def do_cell(cid):
        presim('cell', cid)
        while True:
            ang = teach_one(cli, f'把方块放在 {cid} 号格中心，夹爪张开，压到两指夹住方块中部的高度')
            if ang is None:
                return
            if verify(ang, f'格 {cid}'):
                data['cells'][cid] = ang
                save(a.file, data)
                return
            if input('    回车重教这个点 / q 放弃 ').strip().lower() == 'q':
                return

    def do_zone(z, idx):
        presim('zone', z, idx)
        pts = data['zones'].setdefault(z, [])
        while len(pts) <= idx:
            pts.append([])
        while True:
            ang = teach_one(cli, f'把夹爪摆到 {z} 区上空的放置位置（空中即可，离桌面 3-5 cm，松爪后方块直接掉下去；'
                                 f'手腕姿态尽量和取物点一样自然：J4 接近 0、J5 在 ±40° 内）')
            if ang is None:
                return
            if verify(ang, f'{z} 区放置点'):
                del pts[idx + 1:]          # 空中放置每区只用 1 个点，多余的旧点删掉
                pts[idx] = ang
                save(a.file, data)
                return
            if input('    回车重教这个点 / q 放弃 ').strip().lower() == 'q':
                return

    if a.cmd == 'cell':
        do_cell(int(a.args[0]))
    elif a.cmd == 'zone':
        do_zone(a.args[0], int(a.args[1]) if len(a.args) > 1 else 0)
    elif a.cmd == 'all':
        for cid in range(1, 7):
            print(f'\n===== 取物点 {cid}/6 =====')
            do_cell(cid)
        for z, n in ZONES_DEFAULT.items():
            for i in range(n):
                print(f'\n===== 放置点 {z} {i + 1}/{n} =====')
                do_zone(z, i)
        show(data)
    elif a.cmd == 'goto':
        kind = a.args[0]
        if kind == 'cell':
            ang = data['cells'].get(int(a.args[1]))
        elif kind == 'home':
            ang = data['home_deg']
        else:
            ang = data['zones'][a.args[1]][int(a.args[2]) if len(a.args) > 2 else 0]
        if not ang:
            print('该点还没示教')
            return
        if kind == 'home':
            input(f'即将低速({a.speed}%)回零 {ang}，确认周围无人 [回车]')
            print(cli.goto(ang, a.speed))
            return
        up = list(ang)
        up[1] -= 25.0
        input(f'即将低速({a.speed}%)先到上方点 {[round(x, 1) for x in up]} 再下到 {ang}，确认周围无人 [回车]')
        print(cli.goto(up, a.speed))
        print(cli.goto(ang, a.speed))
        input('到位。回车抬回上方点并回零 ')
        print(cli.goto(up, a.speed))
        print(cli.goto(data['home_deg'], a.speed))


if __name__ == '__main__':
    main()
