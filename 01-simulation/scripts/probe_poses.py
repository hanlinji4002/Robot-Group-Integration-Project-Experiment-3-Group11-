#!/usr/bin/env python3
# 【讲解】调试工具：每 0.4 s 读一次 Gazebo 的 /world/sort_world/pose/info，把机械臂 link6 / 两个指垫 / 方块的真实位姿写成 CSV，
# 用来对照运动学模型（FK）和仿真里的实际位置。用法：python3 probe_poses.py out.csv 秒数 名字1 名字2 ...
import re
import subprocess
import sys
import time

out, secs, names = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
f = open(out, 'w')
f.write('t,' + ','.join(f'{n}_x,{n}_y,{n}_z' for n in names) + '\n')
t0 = time.time()
while time.time() - t0 < secs:
    txt = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', '/world/sort_world/pose/info'],
                         capture_output=True, text=True, timeout=5).stdout
    row = [f'{time.time() - t0:.2f}']
    for n in names:
        m = re.search(r'pose \{\s*name: "' + re.escape(n) + r'"\s*id: \d+\s*position \{\s*x: ([-\d.e]+)\s*y: ([-\d.e]+)\s*z: ([-\d.e]+)', txt)
        row += [m.group(1), m.group(2), m.group(3)] if m else ['', '', '']
    f.write(','.join(row) + '\n')
    f.flush()
    time.sleep(0.4)
