#!/usr/bin/env python3
# 【讲解】任务控制节点 —— 一个"表驱动"的有限状态机（验收要求：BehaviorTree 或状态机）。
# 状态与转移关系写在 config/state_machine.yaml 里，本文件只实现每个状态"做什么、返回哪个结果标签"，
# 下一个状态由 yaml 查表得到。状态：INIT → PARK → SCAN → SELECT → PICK_PLACE → (SCAN | HANDLE_FAIL) … → FINISH/ABORT。
# 输入：/detections（vision_msgs/Detection2DArray，识别节点）+ /camera/camera_info（相机内参）
# 输出：调用 /pick_place 动作（抓取服务器），写 5 份日志：task_state.log / detections.csv / grasp_results.csv / errors.log / summary.txt
# 关键算法：检测框中心像素 → 桌面坐标（aruco 模式：画面里 4 个定位码解出的单应矩阵；camera_pose 模式：相机位姿+内参射线求交）
#           → 最近网格（容差内）→ 类别 → 分类区域。
# 异常处理（验收要求至少 2 类）：空网格跳过、未识别物体(unknown/低置信度)跳过、不可达(plan 失败)跳过、夹取失败重试一次后跳过、通信/控制器故障安全停止。
# 讲解要点：任务开始后不需要任何人工输入——类别来自识别，网格来自几何映射，放置区域来自 class→zone 表。
"""桌面物体自动分类整理：任务状态机节点。"""
import os
import threading
import time
from collections import Counter, defaultdict
from datetime import datetime

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from mecharm_sort.grid_locator import GridLocator
from mecharm_sort.gz_truth import GzTruth
from mecharm_sort.kinematics import rpy_mat
from mecharm_sort.planner import Geometry
from mecharm_sort_interfaces.action import PickPlace


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        for name, default in [
            ('grid_config', ''), ('state_machine_config', ''), ('log_dir', '~/exp3_logs/latest'),
            ('object_classes', ['green', 'red', 'blue']),   # 识别类别
            ('object_zones', ['left', 'front', 'right']),   # 对应分类区域：绿→左，红→前，蓝→右
            ('detections_topic', '/detections'), ('camera_info_topic', '/camera/camera_info'),
            ('debug_image_topic', '/detections/image'), ('image_topic', '/camera/image_raw'),
            # 网格定位方式：aruco = 每次扫描从画面里的 4 个 ArUco 码算像素→桌面映射（相机可任意摆放，真机用）；
            #               camera_pose = 用 grid.yaml 里写死的相机位姿 + 内参（只适合相机完全固定的仿真）
            ('grid_mode', 'aruco'), ('min_markers', 3), ('max_grid_residual', 0.012),
            # 视差修正：方块顶面比桌面高，斜视时会投影到更远处；用单应矩阵估计相机位置后把点拉回。hfov 只需大概
            ('camera_hfov_deg', 65.0), ('object_center_height', 0.018),
            # 感知测试模式：不连机械臂，循环"定位网格 + 识别 + 落格"，把结果叠加发到 /grid/image，日志里打印每格内容
            ('perception_only', False),
            ('sim_check', True), ('sim_pose_topic', '/sim/poses'),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.grid = yaml.safe_load(open(os.path.expanduser(g('grid_config')), encoding='utf-8'))
        self.sm = yaml.safe_load(open(os.path.expanduser(g('state_machine_config')), encoding='utf-8'))
        st = self.sm.get('settings', {})
        self.max_attempts = int(st.get('max_attempts', 2))
        self.max_cycles = int(st.get('max_cycles', 30))
        self.retry_passes = int(st.get('retry_passes', 1))   # 全部处理完后，对夹取失败的格再补抓几轮
        self.scan_frames = int(st.get('scan_frames', 5))
        self.scan_settle = float(st.get('scan_settle_sec', 1.5))
        self.scan_max_retries = int(st.get('scan_max_retries', 5))
        self.scan_retry_wait = float(st.get('scan_retry_wait_sec', 2.0))
        self.min_score = float(st.get('min_score', 0.5))
        self.min_votes = float(st.get('min_vote_ratio', 0.5))
        self.class_to_zone = dict(zip(list(g('object_classes')), list(g('object_zones'))))
        self.geo = Geometry(self.grid)
        self.cells = {int(k): np.array([float(v['x']), float(v['y'])]) for k, v in self.grid['cells'].items()}
        self.zones = self.geo.zones
        self.cell_tol = float(self.grid.get('cell_tolerance', 0.025))
        cam = self.grid['camera']
        self.cam_pos = np.array(cam['xyz'], dtype=float)
        self.cam_R = rpy_mat(*cam['rpy'])
        self.plane_z = float(cam['plane_z'])
        self.sim_check = bool(g('sim_check'))
        self.grid_mode = str(g('grid_mode'))
        self.locator = None
        if self.grid_mode == 'aruco':
            self.locator = GridLocator(self.grid['markers'], self.grid.get('markers_dict', 'DICT_4X4_50'), int(g('min_markers')))
        self.max_grid_residual = float(g('max_grid_residual'))
        self.cam_hfov = float(g('camera_hfov_deg'))
        self.obj_h = float(g('object_center_height'))
        self.scan_retries = 0
        self.last_raw = None
        self.log_dir = os.path.expanduser(g('log_dir'))
        os.makedirs(self.log_dir, exist_ok=True)

        cb = ReentrantCallbackGroup()
        self.K = None
        self.det_lock = threading.Lock()
        self.det_buf = []
        self.det_seq = 0
        self.last_dbg = None
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, g('camera_info_topic'), self._on_info, 10, callback_group=cb)
        self.create_subscription(Detection2DArray, g('detections_topic'), self._on_det, 10, callback_group=cb)
        self.create_subscription(Image, g('debug_image_topic'), self._on_dbg, 5, callback_group=cb)
        if self.grid_mode == 'aruco':
            self.create_subscription(Image, g('image_topic'), self._on_raw, 5, callback_group=cb)
        self.truth = GzTruth(self, g('sim_pose_topic'), 'cube_', cb) if self.sim_check else None
        self.cli = ActionClient(self, PickPlace, '/pick_place', callback_group=cb)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.grid_img_pub = self.create_publisher(Image, '/grid/image', 5)
        self.perception_only = bool(g('perception_only'))

        # 日志文件
        self.f_state = open(os.path.join(self.log_dir, 'task_state.log'), 'a')
        self.f_det = open(os.path.join(self.log_dir, 'detections.csv'), 'w')
        self.f_det.write('scan,cell,class,score,votes,u,v,world_x,world_y\n')
        self.f_res = open(os.path.join(self.log_dir, 'grasp_results.csv'), 'w')
        self.f_res.write('seq,cell,class,zone,slot,attempt,success,failed_stage,message,duration_s,placed_ok,final_x,final_y\n')
        self.f_err = open(os.path.join(self.log_dir, 'errors.log'), 'a')

        # 任务状态
        self.state = self.sm.get('initial', 'INIT')
        self.scan_idx = 0
        self.cycle = 0
        self.seq = 0
        self.cell_status = {}
        self.attempts = defaultdict(int)
        self.skipped = {}          # cell -> 原因
        self.skipped_class = {}    # cell -> 被跳过时看到的类别（内容变了就重新参与）
        self.retry_rounds = 0      # 对夹取失败格的"补抓轮"次数
        self.placed_count = Counter()
        self.sorted_ok = []        # (cell, class, bin)
        self.exceptions = Counter()
        self.current = None        # 当前选中的目标
        self.last_result = None
        self.logged_once = set()
        self.aborted = False
        self.handlers = {
            'INIT': self.s_init, 'PARK': self.s_park, 'SCAN': self.s_scan, 'SELECT': self.s_select,
            'PICK_PLACE': self.s_pick_place, 'HANDLE_FAIL': self.s_handle_fail,
            'FINISH': self.s_finish, 'ABORT': self.s_abort,
        }
        self.worker = threading.Thread(target=self.perception_loop if self.perception_only else self.run, daemon=True)
        self.worker.start()

    # 【讲解】感知测试模式：每秒定位一次网格、拿最近几帧识别结果投票，打印每格内容并发布叠加图；不需要机械臂
    def perception_loop(self):
        self.log('感知测试模式：不动机械臂，持续定位网格 + 识别。看图：ros2 run rqt_image_view rqt_image_view /grid/image')
        t_log = 0.0
        while rclpy.ok():
            time.sleep(0.5)
            if self.locator is not None:
                msg = self.last_raw
                if msg is None:
                    continue
                try:
                    img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:  # noqa: BLE001
                    continue
                ok, why = self.locator.update(img)
                if ok:
                    cam = self.locator.estimate_camera(img.shape[1], img.shape[0], self.cam_hfov)
                    if cam is not None:
                        why += f'；相机位置估计 ({cam[0]:.2f}, {cam[1]:.2f}, 高 {cam[2]:.2f})'
            else:
                ok, why = (self.K is not None), '相机位姿模式'
            with self.det_lock:
                frames = list(self.det_buf[-self.scan_frames:])
            status, lines = ({}, [])
            if ok and frames:
                status, lines, _ = self.vote_cells(frames)
            if time.time() - t_log > 2.0:
                t_log = time.time()
                self.log(('网格定位：' + why) if ok else ('找不到网格：' + why))
                if lines:
                    self.log('；'.join(lines))
            if self.last_dbg is not None:
                try:
                    dbg = self.bridge.imgmsg_to_cv2(self.last_dbg, desired_encoding='bgr8')
                    if self.locator is not None:
                        dbg = self._draw_grid(dbg, cells=ok)
                    y0 = 24
                    cv2.putText(dbg, ('GRID OK  ' if ok else 'GRID NOT FOUND  ') + why, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0) if ok else (0, 0, 255), 2, cv2.LINE_AA)
                    for i, line in enumerate(lines):
                        cv2.putText(dbg, line, (8, y0 + 22 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
                    out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                    out.header = self.last_dbg.header
                    self.grid_img_pub.publish(out)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f'叠加图失败: {e}', throttle_duration_sec=5.0)

    # 【讲解】调试图可能是缩小过的（识别节点 debug_scale），按与原图的比例缩放定位码/格心的像素坐标再画
    def _draw_grid(self, img, cells=True):
        if self.last_raw is None or self.locator is None:
            return img
        sx = img.shape[1] / float(self.last_raw.width)
        sy = img.shape[0] / float(self.last_raw.height)
        if abs(sx - 1) < 1e-3 and abs(sy - 1) < 1e-3:
            return self.locator.draw(img, {cid: tuple(c) for cid, c in self.cells.items()} if cells else None)
        big = cv2.resize(img, (self.last_raw.width, self.last_raw.height))
        big = self.locator.draw(big, {cid: tuple(c) for cid, c in self.cells.items()} if cells else None)
        return cv2.resize(big, (img.shape[1], img.shape[0]))

    # ---------- 日志 ----------
    def log(self, text, level='info'):
        # rclpy 不允许同一行代码用不同级别打日志，所以分三行写
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self.f_state.write(f'{datetime.now().isoformat(timespec="milliseconds")} [{self.state}] {text}\n')
        self.f_state.flush()
        self.status_pub.publish(String(data=f'[{self.state}] {text}'))

    # 【讲解】异常统一记录：类型 + 说明，写 errors.log 并计数（summary 里汇总）
    def exception(self, kind, text, once_key=None):
        if once_key is not None:
            if once_key in self.logged_once:
                return
            self.logged_once.add(once_key)
        self.exceptions[kind] += 1
        self.f_err.write(f'{datetime.now().isoformat(timespec="milliseconds")} [{kind}] {text}\n')
        self.f_err.flush()
        self.log(f'异常[{kind}]：{text}', 'warn')

    # ---------- 订阅回调 ----------
    def _on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.log(f'相机内参 fx={self.K[0]:.1f} fy={self.K[1]:.1f} cx={self.K[2]:.1f} cy={self.K[3]:.1f} ({msg.width}x{msg.height})')

    def _on_det(self, msg):
        with self.det_lock:
            self.det_seq += 1
            self.det_buf.append(msg)
            if len(self.det_buf) > 30:
                self.det_buf.pop(0)

    def _on_dbg(self, msg):
        self.last_dbg = msg

    def _on_raw(self, msg):
        self.last_raw = msg

    # ---------- 几何：像素 → 世界 ----------
    # 【讲解】像素 (u,v) → 光学坐标系射线 → 传感器坐标系 (x前 y左 z上) → 世界 → 与 z=plane_z 平面求交
    def pixel_to_world(self, u, v):
        if self.locator is not None:
            return self.locator.pixel_to_world(u, v, height=self.obj_h)
        fx, fy, cx, cy = self.K
        d_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_sensor = np.array([d_opt[2], -d_opt[0], -d_opt[1]])
        d_w = self.cam_R @ d_sensor
        if abs(d_w[2]) < 1e-6:
            return None
        t = (self.plane_z - self.cam_pos[2]) / d_w[2]
        if t <= 0:
            return None
        return self.cam_pos + t * d_w

    def world_to_cell(self, xy):
        best, best_d = None, self.cell_tol
        for cid, c in self.cells.items():
            d = float(np.linalg.norm(c - xy[:2]))
            if d < best_d:
                best, best_d = cid, d
        return best

    # ---------- 状态机引擎 ----------
    # 【讲解】主循环：执行当前状态的处理函数得到结果标签，再到 yaml 里查"这个状态遇到这个结果转到哪"
    def run(self):
        time.sleep(1.0)
        states = self.sm['states']
        terminal = set(self.sm.get('terminal', ['FINISH', 'ABORT']))
        try:
            while rclpy.ok():
                handler = self.handlers[self.state]
                outcome = handler()
                if self.state in terminal:
                    break
                nxt = states.get(self.state, {}).get(outcome)
                if nxt is None:
                    self.exception('state_machine', f'状态 {self.state} 没有定义结果 {outcome} 的转移，进入 ABORT')
                    nxt = 'ABORT'
                self.log(f'{self.state} --{outcome}--> {nxt}')
                self.state = nxt
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'状态机异常：{e!r}')
            self.exception('internal', repr(e))
            try:
                self.state = 'ABORT'
                self.s_abort()
            except Exception:  # noqa: BLE001
                pass
        finally:
            for f in (self.f_state, self.f_det, self.f_res, self.f_err):
                try:
                    f.close()
                except Exception:  # noqa: BLE001
                    pass
            self.get_logger().info('任务节点退出')
            rclpy.try_shutdown()

    # ---------- 各状态 ----------
    def s_init(self):
        self.log('等待抓取动作服务器 /pick_place ...')
        if not self.cli.wait_for_server(timeout_sec=90):
            self.exception('comm', '抓取动作服务器未上线')
            return 'fail'
        t0 = time.time()
        while ((self.K is None and self.locator is None) or self.det_seq == 0) and time.time() - t0 < 60:
            time.sleep(0.2)
        if self.K is None and self.locator is None:
            self.exception('comm', '收不到相机内参 /camera/camera_info')
            return 'fail'
        if self.det_seq == 0:
            self.exception('comm', '收不到识别结果 /detections')
            return 'fail'
        if self.truth is not None:
            t0 = time.time()
            while not self.truth.received and time.time() - t0 < 10:
                time.sleep(0.2)
            if not self.truth.received:
                self.log('未收到仿真真值 /sim/poses，放置结果将无法核对', 'warn')
        self.log(f'初始化完成：网格 {sorted(self.cells)}，类别→区域 {self.class_to_zone}，网格定位={self.grid_mode}，状态机 {list(self.sm["states"])}')
        return 'ok'

    def s_park(self):
        res = self.call_action('park')
        if res is None or not res.success:
            self.exception('controller', f'机械臂无法回零：{res.message if res else "无响应"}')
            return 'fail'
        return 'ok'

    # 【讲解】扫描：等画面稳定，连收 N 帧检测，逐帧把每个检测框映射到网格并投票，得到每格的类别/置信度
    # 【讲解】多帧投票：把每帧每个检测框映射到网格，按格统计类别 → {格: (类别, 置信度, 票数)}，以及打印用的文字
    def vote_cells(self, frames, write_csv=False):
        votes = {cid: [] for cid in self.cells}
        outside = 0
        for msg in frames:
            for det in msg.detections:
                if not det.results:
                    continue
                cls, score = det.results[0].hypothesis.class_id, det.results[0].hypothesis.score
                u, v = det.bbox.center.position.x, det.bbox.center.position.y
                pw = self.pixel_to_world(u, v)
                cid = self.world_to_cell(pw) if pw is not None else None
                if cid is None:
                    outside += 1
                    continue
                votes[cid].append((cls, score, u, v, pw))
        status, lines = {}, []
        for cid in sorted(self.cells):
            vs = votes[cid]
            if not vs:
                status[cid] = ('empty', 0.0, 0)
                if write_csv:
                    self.f_det.write(f'{self.scan_idx},{cid},empty,0,0,,,,\n')
                lines.append(f'格{cid}: 空')
                continue
            cnt = Counter(v[0] for v in vs)
            cls, n = cnt.most_common(1)[0]
            sel = [v for v in vs if v[0] == cls]
            score = float(np.mean([v[1] for v in sel]))
            u, v_, pw = sel[-1][2], sel[-1][3], sel[-1][4]
            if n < self.min_votes * len(frames):
                cls = 'unstable'
            status[cid] = (cls, score, n)
            if write_csv:
                self.f_det.write(f'{self.scan_idx},{cid},{cls},{score:.3f},{n},{u:.1f},{v_:.1f},{pw[0]:.4f},{pw[1]:.4f}\n')
            lines.append(f'格{cid}: {cls} {score:.2f} ({n}/{len(frames)}帧)')
        return status, lines, outside

    # 【讲解】aruco 模式先在最新画面里找 4 个定位码算映射；找不到就等一会再试（相机被挡/没对准），多次失败才判故障
    def locate_grid(self):
        t0 = time.time()
        last = ''
        while time.time() - t0 < 3.0:
            msg = self.last_raw
            if msg is not None:
                try:
                    img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception as e:  # noqa: BLE001
                    return False, f'图像转换失败 {e}'
                ok, why = self.locator.update(img)
                if ok and (self.locator.residual or 0) <= self.max_grid_residual:
                    cam = self.locator.estimate_camera(img.shape[1], img.shape[0], self.cam_hfov)
                    if cam is not None:
                        why += f'；相机位置估计 ({cam[0]:.2f}, {cam[1]:.2f}, 高 {cam[2]:.2f}) m'
                    return True, why
                last = why if ok else why
                if ok:
                    last = why + f'，残差超过 {self.max_grid_residual * 1000:.0f} mm'
            time.sleep(0.1)
        return False, last or '没有收到相机图像'

    def s_scan(self):
        self.scan_idx += 1
        time.sleep(self.scan_settle)
        if self.locator is not None:
            ok, why = self.locate_grid()
            if not ok:
                self.scan_retries += 1
                self.exception('grid_not_found', f'扫描#{self.scan_idx} 找不到网格：{why}（第 {self.scan_retries} 次）')
                if self.scan_retries > self.scan_max_retries:
                    return 'fail'
                time.sleep(self.scan_retry_wait)
                return 'retry'
            self.scan_retries = 0
            self.log(f'网格定位：{why}')
        with self.det_lock:
            start_seq = self.det_seq
        t0 = time.time()
        while self.det_seq - start_seq < self.scan_frames and time.time() - t0 < 15:
            time.sleep(0.05)
        with self.det_lock:
            frames = list(self.det_buf[-self.scan_frames:])
        if len(frames) < self.scan_frames:
            self.exception('comm', f'扫描超时：15 s 内只收到 {len(frames)} 帧识别结果')
            return 'fail'
        status, lines, outside = self.vote_cells(frames, write_csv=True)
        self.f_det.flush()
        self.cell_status = status
        # 【讲解】格子内容变了就重新参与排队：被跳过的格变空 → 清掉跳过标记和失败计数（原物体已拿走，下次放进来的是新物体）；
        # 被跳过时是"未识别"，现在变成可识别类别 → 也重新参与。这样老师中途拿走/放回方块，程序仍按 1→6 顺序处理
        for cid, (cls, score, n) in status.items():
            if cls == 'empty':
                if cid in self.skipped:
                    self.log(f'网格 {cid} 已清空，取消跳过标记（原因 {self.skipped[cid]}）')
                    self.skipped.pop(cid, None)
                    self.skipped_class.pop(cid, None)
                self.attempts[cid] = 0
            elif cid in self.skipped and self.skipped_class.get(cid) not in (None, cls) and cls in self.class_to_zone:
                self.log(f'网格 {cid} 内容由 {self.skipped_class.get(cid)} 变为 {cls}，取消跳过标记')
                self.skipped.pop(cid, None)
                self.skipped_class.pop(cid, None)
                self.attempts[cid] = 0
        self.log(f'扫描#{self.scan_idx}：' + '；'.join(lines) + (f'；网格外检测 {outside // len(frames)} 个/帧（忽略）' if outside else ''))
        if self.last_dbg is not None:
            try:
                img = self.bridge.imgmsg_to_cv2(self.last_dbg, desired_encoding='bgr8')
                if self.locator is not None:
                    img = self._draw_grid(img)
                cv2.imwrite(os.path.join(self.log_dir, f'scan_{self.scan_idx:02d}.png'), img)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'保存扫描截图失败: {e}')
        return 'ok'

    # 【讲解】选目标：按网格编号从小到大，找第一个"已识别、置信度够、没被跳过"的格子；顺带把空格/未识别记成异常
    def s_select(self):
        if self.cycle >= self.max_cycles:
            self.log(f'达到最大循环次数 {self.max_cycles}，结束')
            return 'none'
        outcome = self._select_pass()
        if outcome == 'none':
            # 【讲解】1→6 都处理完了：对因夹取/轨迹失败被跳过的格再补抓一轮（未识别/不可达的不补），最多 retry_passes 轮
            retry = [c for c, r in self.skipped.items() if r in ('grasp_failed', 'trajectory_failed')]
            if retry and self.retry_rounds < self.retry_passes:
                self.retry_rounds += 1
                for c in retry:
                    self.skipped.pop(c, None)
                    self.skipped_class.pop(c, None)
                    self.attempts[c] = 0
                self.log(f'第 {self.retry_rounds} 轮补抓：重新尝试网格 {sorted(retry)}')
                outcome = self._select_pass()
        return outcome

    def _select_pass(self):
        for cid in sorted(self.cells):
            cls, score, n = self.cell_status.get(cid, ('empty', 0.0, 0))
            if cid in self.skipped:
                continue
            if cls == 'empty':
                self.exception('empty_grid', f'网格 {cid} 为空，跳过', once_key=('empty', cid))
                continue
            if cls in ('unknown', 'unstable') or cls not in self.class_to_zone or score < self.min_score:
                reason = f'网格 {cid} 物体未识别（类别 {cls}，置信度 {score:.2f}），跳过'
                self.exception('unknown_object', reason, once_key=('unknown', cid, cls))
                self.skipped[cid] = 'unknown_object'
                self.skipped_class[cid] = cls
                continue
            zone_id = self.class_to_zone[cls]
            slot = int(self.placed_count[zone_id])
            cube = self.truth.nearest(self.cells[cid], 0.03) if self.truth is not None else None
            self.current = {'cell': cid, 'class': cls, 'score': score, 'zone': zone_id, 'slot': slot, 'cube': cube}
            self.cycle += 1
            self.log(f'第 {self.cycle} 轮目标：按 1→6 顺序选中网格 {cid} {cls}（{score:.2f}）→ 区域 {zone_id} 槽位 {slot}' + (f'，真值物体 {cube}' if cube else ''))
            return 'ok'
        self.log('没有可执行目标（全部处理完成或剩余均为异常格）')
        return 'none'

    # 【讲解】调用抓取动作并等结果；成功后用仿真真值核对物体是否真的落在分类区域内。
    # 动作失败但真值显示物体已经落在目标区域（例如放置时被顶住、安全返回时在放置点上方释放）→ 也算分类完成
    def s_pick_place(self):
        c = self.current
        self.attempts[c['cell']] += 1
        self.seq += 1
        res = self.call_action('pick_place', grid_id=c['cell'], object_class=c['class'], zone_id=c['zone'], slot=c['slot'])
        self.last_result = res
        ok = res is not None and res.success
        placed_ok, fx, fy = '', '', ''
        if self.truth is not None and c['cube']:
            time.sleep(0.5)
            p = self.truth.get(c['cube'])
            if p is not None:
                inside = self.geo.in_zone(c['zone'], p[:2], margin=0.01)
                placed_ok, fx, fy = ('1' if inside else '0'), f'{p[0]:.4f}', f'{p[1]:.4f}'
                if inside and not ok:
                    self.exception('recovered', f'网格 {c["cell"]} 动作失败[{res.failed_stage if res else "comm"}]，但 {c["cube"]} 已落在区域 {c["zone"]} 内，按完成处理')
                    ok = True
                if ok and not inside:
                    self.exception('misplaced', f'{c["cube"]} 释放后不在区域 {c["zone"]} 内：({p[0]:.3f},{p[1]:.3f})')
        if ok:
            self.attempts[c['cell']] = 0
            self.placed_count[c['zone']] += 1
            if placed_ok != '0':
                self.sorted_ok.append((c['cell'], c['class'], c['zone']))
            self.log(f'网格 {c["cell"]} {c["class"]} → {c["zone"]} 完成（{res.duration:.1f} s）' + (f'，落点核对 {"通过" if placed_ok == "1" else "不通过"}' if placed_ok else ''))
        self.f_res.write(f'{self.seq},{c["cell"]},{c["class"]},{c["zone"]},{c["slot"]},{self.attempts[c["cell"]]},{1 if ok else 0},'
                         f'{"" if res is None else res.failed_stage},"{"无响应" if res is None else res.message}",'
                         f'{0 if res is None else res.duration:.1f},{placed_ok},{fx},{fy}\n')
        self.f_res.flush()
        return 'ok' if ok else 'fail'

    # 【讲解】失败分流：不可达 → 跳过该格；夹空 → 重试一次再跳过；控制器/通信故障 → 安全停止
    def s_handle_fail(self):
        c, res = self.current, self.last_result
        stage = res.failed_stage if res is not None else 'comm'
        msg = res.message if res is not None else '动作服务器无响应'
        if stage == 'plan':
            self.exception('unreachable', f'网格 {c["cell"]} 目标不可达/超限：{msg}，跳过')
            self.skipped[c['cell']] = 'unreachable'
            self.skipped_class[c['cell']] = c['class']
            return 'skip'
        if stage in ('grasp_verify', 'controller'):
            # 夹空或轨迹执行失败（例如下降时被物体顶住）都算一次抓取失败：机械臂已回安全位，重试一次，再失败就放弃该格
            kind = 'grasp_failed' if stage == 'grasp_verify' else 'trajectory_failed'
            if self.attempts[c['cell']] < self.max_attempts:
                self.exception(kind, f'网格 {c["cell"]} 抓取失败[{stage}]：{msg}，重试（第 {self.attempts[c["cell"]] + 1} 次）')
                return 'retry'
            self.exception(kind, f'网格 {c["cell"]} 抓取失败 {self.attempts[c["cell"]]} 次[{stage}]：{msg}，暂时跳过（其余格处理完后补抓）')
            self.skipped[c['cell']] = kind
            self.skipped_class[c['cell']] = c['class']
            return 'skip'
        # 通信故障（动作服务器无响应/内部异常）：无法保证机械臂状态，安全停止
        self.exception('comm', f'网格 {c["cell"]} 执行故障[{stage}]：{msg}，安全停止')
        return 'abort'

    def s_finish(self):
        self.write_summary('FINISHED')
        return 'done'

    def s_abort(self):
        self.aborted = True
        self.log('安全停止：机械臂回零', 'error')
        self.call_action('park')
        self.write_summary('ABORTED')
        return 'done'

    # ---------- 动作调用 ----------
    def call_action(self, command, grid_id=0, object_class='', zone_id='', slot=0, timeout=240.0):
        goal = PickPlace.Goal()
        goal.command, goal.grid_id, goal.object_class, goal.zone_id, goal.slot = command, int(grid_id), object_class, zone_id, int(slot)
        fut = self.cli.send_goal_async(goal, feedback_callback=lambda fb: self.log(f'  动作反馈 {fb.feedback.step}/{fb.feedback.total_steps} {fb.feedback.stage}'))
        t0 = time.time()
        while not fut.done():
            time.sleep(0.05)
            if time.time() - t0 > 15:
                self.log('动作目标发送超时', 'error')
                return None
        gh = fut.result()
        if not gh.accepted:
            self.log('动作目标被拒绝', 'error')
            return None
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done():
            time.sleep(0.05)
            if time.time() - t0 > timeout:
                self.log('动作执行超时', 'error')
                return None
        return rf.result().result

    # ---------- 汇总 ----------
    def write_summary(self, status):
        n_cls = Counter(cls for _, cls, _ in self.sorted_ok)
        lines = [
            f'任务状态：{status}',
            f'扫描次数：{self.scan_idx}，抓取轮次：{self.cycle}',
            f'成功分类物体：{len(self.sorted_ok)} 个（' + '，'.join(f'{k} {v}' for k, v in n_cls.items()) + '）',
            '各区域：' + '，'.join(f'{b} {n}' for b, n in self.placed_count.items()),
            '跳过的网格：' + ('，'.join(f'{c}({r})' for c, r in self.skipped.items()) or '无'),
            '异常统计：' + ('，'.join(f'{k} {v}' for k, v in self.exceptions.items()) or '无'),
            f'日志目录：{self.log_dir}',
        ]
        text = '\n'.join(lines)
        with open(os.path.join(self.log_dir, 'summary.txt'), 'w') as f:
            f.write(text + '\n')
        self.log('===== 任务汇总 =====\n' + text)


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
