#!/usr/bin/env python3
# 【讲解】生成两样东西：
#   1) markers/aruco_<id>.png    仿真世界里贴在桌面上的 4 个 ArUco 码贴图（gen_world.py 引用）
#   2) grid_sheet_A4.png/.pdf    真机用的网格纸：3×2 网格黑线 + 四角 ArUco 码 + 编号 + 对准标记，1:1 比例，A4 横向打印
# 网格、码的位置全部读 grid.yaml，保证仿真、真机、程序三者一致。用法：python3 make_markers.py [grid.yaml]
import os
import sys

import cv2
import numpy as np
import yaml

DPI = 300
MM = DPI / 25.4  # 像素/毫米


def main(grid_path):
    cfg = yaml.safe_load(open(grid_path, encoding='utf-8'))
    here = os.path.dirname(os.path.abspath(__file__))
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg['markers_dict']))
    out_dir = os.path.join(here, '..', 'markers')
    os.makedirs(out_dir, exist_ok=True)
    for mid in cfg['markers']:
        img = cv2.aruco.generateImageMarker(d, int(mid), 400)
        img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)  # 白边（quiet zone）
        cv2.imwrite(os.path.join(out_dir, f'aruco_{mid}.png'), img)
    print(f'已生成 {len(cfg["markers"])} 个码贴图 → {os.path.abspath(out_dir)}')

    # ---- 网格纸（A4 横向 297×210 mm）：纸的坐标系 = 桌面坐标系平移；x 向前（纸的短边方向向上），y 向左 ----
    csx, csy = cfg['cell_size']
    cells = {int(k): (v['x'], v['y']) for k, v in cfg['cells'].items()}
    xs = [c[0] for c in cells.values()]
    ys = [c[1] for c in cells.values()]
    x0, x1 = min(xs) - csx / 2, max(xs) + csx / 2
    y0, y1 = min(ys) - csy / 2, max(ys) + csy / 2
    W, H = int(297 * MM), int(210 * MM)
    sheet = np.full((H, W), 255, np.uint8)
    # 纸面映射：桌面 x=sheet_x0 对应纸下边缘上方 margin；y=0 对应纸中线
    sheet_x0 = float(cfg.get('sheet_x0', 0.04))   # 纸的近边在桌面上的 x（距底座中心）
    margin_px = int(10 * MM)

    def to_px(x, y):
        u = W / 2 - y * 1000 * MM
        v = H - margin_px - (x - sheet_x0) * 1000 * MM
        return int(round(u)), int(round(v))

    # 网格线（2 mm 粗）
    t = max(2, int(2 * MM))
    for i in range(4):
        y = y0 + i * csy
        cv2.line(sheet, to_px(x0, y), to_px(x1, y), 0, t)
    for i in range(3):
        x = x0 + i * csx
        cv2.line(sheet, to_px(x, y0), to_px(x, y1), 0, t)
    # 格编号（格子角落小字，不放中心）
    for cid, (x, y) in cells.items():
        u, v = to_px(x + csx / 2 - 0.006, y + csy / 2 - 0.004)
        cv2.putText(sheet, str(cid), (u, v), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3, cv2.LINE_AA)
    # ArUco 码（含白边）
    for mid, m in cfg['markers'].items():
        s = m['size']
        img = cv2.aruco.generateImageMarker(d, int(mid), int(s * 1000 * MM))
        u, v = to_px(m['x'] + s / 2, m['y'] + s / 2)   # 码的"左上角"= 桌面 (+x,+y) 角
        h, w = img.shape
        sheet[v:v + h, u:u + w] = img
        cv2.putText(sheet, f'id{mid}', (u, v + h + int(5 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2, cv2.LINE_AA)
    # 对准标记：纸下边中点 = 机器人正前方；标注纸近边到底座中心的距离
    cx, cy = to_px(sheet_x0, 0.0)
    cv2.line(sheet, (cx, cy), (cx, cy - int(15 * MM)), 0, t)
    cv2.putText(sheet, f'robot base center: {sheet_x0 * 1000:.0f} mm below this edge, on this line',
                (cx - int(75 * MM), cy - int(20 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
    cv2.putText(sheet, 'Exp3 grid sheet  A4 landscape, print at 100% (no scaling)   cells 50x60 mm',
                (int(10 * MM), int(12 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
    png = os.path.join(here, 'grid_sheet_A4.png')
    cv2.imwrite(png, sheet)
    try:
        from PIL import Image
        Image.open(png).save(os.path.join(here, 'grid_sheet_A4.pdf'), 'PDF', resolution=DPI)
        print('已生成 grid_sheet_A4.png / .pdf（300 dpi，A4 横向，1:1 打印）')
    except Exception as e:  # noqa: BLE001
        print('已生成 grid_sheet_A4.png（PDF 未生成：', e, '）')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'grid.yaml'))
