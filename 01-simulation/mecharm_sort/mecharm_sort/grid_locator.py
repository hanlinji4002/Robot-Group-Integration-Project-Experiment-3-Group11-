#!/usr/bin/env python3
# 【讲解】网格定位器：相机位置不固定，所以不标定相机，而是让程序每次扫描时自己在画面里找网格。
# 做法：网格纸四角印 4 个 ArUco 码（DICT_4X4_50，编号 0-3），它们在桌面坐标系（机器人底座为原点）里的位置写在 grid.yaml。
# 每帧：检测 ArUco → 拿到每个码中心的像素坐标 → 与已知桌面坐标配对 → 解单应矩阵 H（像素 → 桌面平面）。
# 之后任何检测框中心 (u,v) 都能用 H 换算成桌面 (x,y)，再判断落在哪个网格。相机怎么摆、有没有挪过都无所谓，
# 只要 4 个码里至少看见 3 个（4 个 → 单应，3 个 → 仿射近似）。
# 用码中心而不用四角，是为了对码的粘贴方向不敏感（贴歪、贴转 90° 都行）。
"""ArUco 网格定位：图像 → 像素到桌面平面的映射。"""
import math

import cv2
import numpy as np

_DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50, 'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50, 'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


class GridLocator:
    def __init__(self, markers, dictionary='DICT_4X4_50', min_markers=3):
        """markers: {id: {'x','y','size'}} 桌面坐标（m）。"""
        self.markers = {int(k): (float(v['x']), float(v['y']), float(v.get('size', 0.03))) for k, v in markers.items()}
        self.min_markers = int(min_markers)
        d = cv2.aruco.getPredefinedDictionary(_DICTS[dictionary])
        params = cv2.aruco.DetectorParameters()
        # 放宽检测参数：远处的码在画面里可能只有 25 px 宽（默认参数会漏掉），允许更小的四边形、更细的自适应阈值窗口
        params.minMarkerPerimeterRate = 0.01
        params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax, params.adaptiveThreshWinSizeStep = 3, 53, 4
        params.polygonalApproxAccuracyRate = 0.05
        params.errorCorrectionRate = 0.8
        params.perspectiveRemovePixelPerCell = 8
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(d, params)
        self.H = None          # 3x3 单应（像素 → 桌面）
        self.mode = None       # 'homography' | 'affine'
        self.found = {}        # id → 像素中心
        self.residual = None   # 已知点回投残差（m）
        self.cam = None        # 由单应矩阵估计的相机位置（桌面坐标系，m）

    # 【讲解】检测一帧：返回 (是否成功, 说明文字)。成功后 self.H 可用
    def update(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        corners, ids, _ = self.detector.detectMarkers(gray)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                i = int(i)
                if i in self.markers:
                    found[i] = c.reshape(4, 2).mean(axis=0)
        self.found = found
        if len(found) < self.min_markers:
            self.H, self.mode = None, None
            return False, f'只看到 {len(found)} 个网格定位码 {sorted(found)}，至少需要 {self.min_markers} 个'
        src = np.array([found[i] for i in sorted(found)], dtype=np.float64)
        dst = np.array([self.markers[i][:2] for i in sorted(found)], dtype=np.float64)
        if len(found) >= 4:
            H, _ = cv2.findHomography(src, dst, 0)
            self.mode = 'homography'
        else:
            A = cv2.getAffineTransform(src[:3].astype(np.float32), dst[:3].astype(np.float32))
            H = np.vstack([A, [0, 0, 1]])
            self.mode = 'affine'
        if H is None:
            self.H = None
            return False, '单应矩阵求解失败'
        self.H = H
        proj = self.pixel_to_world_batch(src)
        self.residual = float(np.sqrt(((proj - dst) ** 2).sum(axis=1)).mean())
        return True, f'定位码 {sorted(found)}（{self.mode}），回投残差 {self.residual * 1000:.1f} mm'

    def pixel_to_world_batch(self, uv):
        pts = np.hstack([np.asarray(uv, dtype=np.float64), np.ones((len(uv), 1))])
        w = pts @ self.H.T
        return w[:, :2] / w[:, 2:3]

    # 【讲解】单应矩阵只对"桌面上的点"准确。方块顶面比桌面高 2.5 cm，相机斜着看时顶面中心会投影到桌面上更远的位置
    # （相机越低偏得越多，30° 仰角时偏 4 cm，比半个格子还大）。用单应矩阵反推相机位置，再把点沿"朝相机方向"拉回来：
    # 真实 xy = 桌面投影点 + (相机地面投影 - 桌面投影点) × (物体高度 / 相机高度)。
    # 相机内参只需大概（按水平视场角估），误差只影响修正量的 10-20%
    def estimate_camera(self, width, height, hfov_deg=65.0):
        if self.H is None:
            self.cam = None
            return None
        f = width / (2.0 * math.tan(math.radians(hfov_deg) / 2))
        K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])
        G = np.linalg.inv(self.H)                      # 桌面 (x,y,1) → 像素
        M = np.linalg.inv(K) @ G
        lam = 1.0 / np.linalg.norm(M[:, 0])
        best = None
        for sign in (1.0, -1.0):
            r1, r2, t = sign * lam * M[:, 0], sign * lam * M[:, 1], sign * lam * M[:, 2]
            R = np.column_stack([r1, r2, np.cross(r1, r2)])
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt
            C = -R.T @ t
            if C[2] > 0.05 and (best is None or C[2] > best[2]):
                best = C
        self.cam = best
        return best

    def pixel_to_world(self, u, v, height=0.0):
        if self.H is None:
            return None
        xy = self.pixel_to_world_batch([[u, v]])[0]
        if height > 0 and getattr(self, 'cam', None) is not None and self.cam[2] > height:
            xy = xy + (self.cam[:2] - xy) * (height / self.cam[2])
        return np.array([xy[0], xy[1], 0.0])

    # 【讲解】把找到的码和格心画到调试图上，方便现场对照
    def draw(self, bgr, cells=None):
        for i, c in self.found.items():
            cv2.circle(bgr, (int(c[0]), int(c[1])), 6, (0, 255, 255), 2)
            cv2.putText(bgr, f'id{i}', (int(c[0]) + 8, int(c[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        if self.H is not None and cells:
            Hinv = np.linalg.inv(self.H)
            for cid, (x, y) in cells.items():
                p = Hinv @ np.array([x, y, 1.0])
                u, v = p[0] / p[2], p[1] / p[2]
                cv2.drawMarker(bgr, (int(u), int(v)), (255, 255, 0), cv2.MARKER_CROSS, 12, 1)
                cv2.putText(bgr, str(cid), (int(u) + 6, int(v) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
        return bgr
