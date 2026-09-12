#!/usr/bin/env python3
# 【讲解】抓取动作服务器（ROS 2 Action：mecharm_sort_interfaces/PickPlace）—— 机械臂端的"固定点抓取程序"。
# 任务节点发来"网格编号 + 区域编号 + 槽位"，这里查 grid.yaml 得到固定取物点/放置点（启动时已预先逆解好），按步骤下发轨迹，并实时反馈进度。
# 步骤：张爪 → 取物点上方 → 下降 → 夹取 → 抬升 → 夹取校验 → 过渡高度 → 转向区域 → 放置点上方 → 下降 → 释放 → 抬升 → 抬回过渡高度 → 转回正前 → 回零。
# 异常：任一路径点无逆解/超限 → failed_stage=plan（目标不可达，任务不执行）；夹空 → grasp_verify（松爪回安全位）；
#       控制器拒绝/超时 → controller（回安全位）。失败时机械臂都先回零（安全位置）再返回结果（验收要求）。
# 讲解要点：本节点只跟 /arm_controller、/hand_controller 两个 FollowJointTrajectory 动作服务器说话，仿真由 ros2_control 提供，
#           真机由实验二的 real_driver 提供，切真机不改这里，只改参数（grasp_check 改成 gripper）。
"""PickPlace 动作服务器：按网格编号执行固定点取物 → 分类放置 → 返回。"""
import math
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mecharm_sort.gz_truth import GzTruth
from mecharm_sort.kinematics import ARM_JOINTS, GRIPPER_JOINT, check_limits

# 真机限位（度，取实验二 arm_common 与 URDF 中较严者），示教点超出即判不可达
REAL_LIMIT_MIN = [-160, -75, -175, -155, -115, -175]
REAL_LIMIT_MAX = [160, 120, 65, 155, 115, 175]


def check_real_limits(q_rad):
    for j, (v, lo, hi) in enumerate(zip(np.degrees(q_rad), REAL_LIMIT_MIN, REAL_LIMIT_MAX)):
        if not (lo <= v <= hi):
            return False, f'{ARM_JOINTS[j]} {v:.1f}° 超出真机限位 [{lo},{hi}]'
    return True, ''
from mecharm_sort.planner import Geometry, Unreachable, compose, plan_cell, plan_slot
from mecharm_sort_interfaces.action import PickPlace


class StepFailed(Exception):
    def __init__(self, stage, message):
        super().__init__(message)
        self.stage, self.message = stage, message


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')
        for name, default in [
            ('grid_config', ''), ('log_dir', '~/exp3_logs/latest'),
            ('tool_tip_offset', 0.063), ('safe_height', 0.05), ('transit_height', 0.08),
            ('place_height', 0.03), ('place_above', 0.06),
            ('pick_tilt', 0.26), ('place_tilt', 0.68), ('jaw_axis', 'y'),
            ('gripper_open', 0.0), ('gripper_close', -0.44),
            ('move_duration', 2.0), ('grip_duration', 1.2),
            ('home_pose_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), ('transit_tilt', 0.26),
            # 夹取校验方式：sim_pose=仿真真值(物体被抬起)；gripper=夹爪读数(真机)；none=不校验
            ('grasp_check', 'sim_pose'), ('lift_check_dz', 0.02), ('gripper_empty_tol', 0.02),
            ('sim_pose_topic', '/sim/poses'),
            # 异常测试钩子：对这些网格的前 N 次夹取"不合爪"（模拟打滑夹空），用来验证夹取失败的处理链
            ('inject_grasp_fail_grids', [0]), ('inject_grasp_fail_times', 1),
            # 示教回放模式（真机）：不做逆解，直接回放 taught_points.yaml 里每格的取物关节角和每个落点的放置关节角；
            # 上方点 = J2 往回收 lift_deg（垂直抬起），过渡点 = J2 再多收 transit_lift_deg
            ('use_taught_joints', False), ('taught_points_file', ''),
            ('lift_deg', 25.0), ('transit_lift_deg', 35.0),
            # 空中放置：放置点本身就教在空中，到点直接松爪扔下（不再"上方点→下降→释放→抬升"）
            ('drop_in_air', False),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        cfg = yaml.safe_load(open(os.path.expanduser(g('grid_config')), encoding='utf-8'))
        self.geo = Geometry(cfg)
        self.cells, self.zones = self.geo.cells, self.geo.zones
        self.tool = float(g('tool_tip_offset'))
        self.safe_h, self.transit_h = float(g('safe_height')), float(g('transit_height'))
        self.place_h, self.place_above = float(g('place_height')), float(g('place_above'))
        self.pick_tilt, self.place_tilt = float(g('pick_tilt')), float(g('place_tilt'))
        self.jaw_axis = str(g('jaw_axis'))
        self.grip_open, self.grip_close = float(g('gripper_open')), float(g('gripper_close'))
        self.move_dur, self.grip_dur = float(g('move_duration')), float(g('grip_duration'))
        self.home_q = np.radians(np.array(g('home_pose_deg'), dtype=float))
        self.transit_tilt = float(g('transit_tilt'))
        self.grasp_check = str(g('grasp_check'))
        self.lift_dz, self.grip_empty_tol = float(g('lift_check_dz')), float(g('gripper_empty_tol'))
        self.inject_grids = [int(v) for v in g('inject_grasp_fail_grids') if int(v) > 0]
        self.inject_times = int(g('inject_grasp_fail_times'))
        self.inject_count = {}
        self.use_taught = bool(g('use_taught_joints'))
        self.taught = None
        if self.use_taught:
            self.taught = yaml.safe_load(open(os.path.expanduser(g('taught_points_file')), encoding='utf-8'))
            self.lift = math.radians(float(g('lift_deg')))
            self.transit_lift = math.radians(float(g('transit_lift_deg')))
        self.drop_in_air = bool(g('drop_in_air'))
        self.cell_plans, self.slot_plans = {}, {}
        self.holding = None   # 夹取校验通过后 = 放置点上方路径点（失败时带物体去那里释放），释放后清空
        self.holding_fallback = None   # 备用释放点 = 取物格过渡点
        self.log_dir = os.path.expanduser(g('log_dir'))
        os.makedirs(self.log_dir, exist_ok=True)

        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.arm_cli = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.hand_cli = ActionClient(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory', callback_group=cb)
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)
        self.joint_q = {}
        self.create_subscription(JointState, '/joint_states', self._on_js, 10, callback_group=cb)
        self.truth = GzTruth(self, g('sim_pose_topic'), 'cube_', cb) if self.grasp_check == 'sim_pose' else None
        self.traj_file = open(os.path.join(self.log_dir, 'trajectory.csv'), 'w')
        self.traj_file.write('stamp,' + ','.join(ARM_JOINTS + [GRIPPER_JOINT]) + '\n')
        self.traj_lock = threading.Lock()
        self.busy = False
        # 启动时同步预算全部路径点（6 格 + 9 个落点），之后每次动作直接查表；算完才上线动作服务器
        self._precompute_plans()
        self.server = ActionServer(self, PickPlace, '/pick_place', execute_callback=self._execute,
                                   goal_callback=self._on_goal, cancel_callback=lambda gh: CancelResponse.ACCEPT,
                                   callback_group=cb)
        self.get_logger().info(f'抓取动作服务器就绪：{len(self.cells)} 格，分类区域 {list(self.zones)}，夹取校验={self.grasp_check}，注入失败格={self.inject_grids}')

    def _plan_params(self):
        return dict(tool_tip_offset=self.tool, safe_height=self.safe_h, transit_height=self.transit_h,
                    place_height=self.place_h, place_above=self.place_above,
                    pick_tilt=self.pick_tilt, place_tilt=self.place_tilt, transit_tilt=self.transit_tilt,
                    jaw_axis=self.jaw_axis)

    # 【讲解】预规划：每个网格的取物侧路径点、每个区域每个槽位的放置侧路径点，各算一次存起来；不可达的也记下来
    # 【讲解】示教模式的"规划"：查表 + J2 抬角。没示教的格/落点视为不可达（任务节点会记 unreachable 并跳过）
    def _taught_cell(self, grid_id):
        cells = self.taught.get('cells', {}) or {}
        degs = cells.get(grid_id, cells.get(str(grid_id)))
        if degs is None or len(degs) != 6:
            raise Unreachable(f'网格 {grid_id} 没有示教取物点')
        pick = np.radians(np.array(degs, dtype=float))
        above, transit = pick.copy(), pick.copy()
        above[1] -= self.lift
        transit[1] -= self.transit_lift
        wp = {'pick': pick, 'above_cell': above, 'transit': transit}
        for name, q in wp.items():
            ok, why = check_real_limits(q)
            if not ok:
                raise Unreachable(f'网格 {grid_id} 示教点 {name} {why}')
        return wp

    def _taught_slot(self, zone_id, slot):
        zones = self.taught.get('zones', {}) or {}
        # 只用真正教过的点（6 个关节角），空占位 [] 一律忽略；同一区域有多个点就轮流用，只有 1 个就一直用它
        slots = [p for p in (zones.get(zone_id) or []) if p is not None and len(p) == 6]
        if not slots:
            raise Unreachable(f'区域 {zone_id} 没有示教放置点')
        degs = slots[slot % len(slots)]
        place = np.radians(np.array(degs, dtype=float))
        above = place.copy()
        if not self.drop_in_air:
            above[1] -= self.lift      # 空中放置时上方点 = 放置点本身，省掉下降/抬升两步
        wp = {'place': place, 'above_zone': above}
        for name, q in wp.items():
            ok, why = check_real_limits(q)
            if not ok:
                raise Unreachable(f'区域 {zone_id} 示教点 {name} {why}')
        return wp

    def _precompute_plans(self):
        t0 = time.time()
        if self.use_taught:
            home = self.taught.get('home_deg')
            if home is not None:
                self.home_q = np.radians(np.array(home, dtype=float))
            self.get_logger().info(f'示教回放模式：{len(self.taught.get("cells", {}))} 个取物点，'
                                   f'{sum(len(v) for v in (self.taught.get("zones", {}) or {}).values())} 个放置点，'
                                   f'抬升 {math.degrees(self.lift):.0f}°，过渡 {math.degrees(self.transit_lift):.0f}°')
        for grid_id in sorted(self.cells):
            try:
                self.cell_plans[grid_id] = self._taught_cell(grid_id) if self.use_taught else plan_cell(self.geo, grid_id, self._plan_params())
            except Unreachable as e:
                self.cell_plans[grid_id] = e
                self.get_logger().warn(f'预规划 网格 {grid_id} 不可达：{e}')
        for zone_id, z in self.zones.items():
            for slot in range(len(z['slots'])):
                try:
                    self.slot_plans[(zone_id, slot)] = self._taught_slot(zone_id, slot) if self.use_taught else plan_slot(self.geo, zone_id, slot, self._plan_params())
                except Unreachable as e:
                    self.slot_plans[(zone_id, slot)] = e
                    self.get_logger().warn(f'预规划 区域 {zone_id} 槽位 {slot} 不可达：{e}')
        n_ok = sum(1 for v in list(self.cell_plans.values()) + list(self.slot_plans.values()) if not isinstance(v, Exception))
        self.get_logger().info(f'预规划完成：{n_ok}/{len(self.cell_plans) + len(self.slot_plans)} 组可达，用时 {time.time() - t0:.1f} s')

    # ---------- 基础设施 ----------
    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_q[n] = p
        with self.traj_lock:
            if not self.traj_file.closed:
                row = [f'{self.joint_q.get(j, float("nan")):.4f}' for j in ARM_JOINTS + [GRIPPER_JOINT]]
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.traj_file.write(f'{t:.3f},' + ','.join(row) + '\n')

    def _on_goal(self, goal):
        if self.busy:
            self.get_logger().warn('上一个动作还在执行，拒绝新目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ---------- 运动指令 ----------
    # 【讲解】发一条单点轨迹给控制器，先查限位再等结果；任何异常抛 StepFailed 让上层统一处理
    def _send_traj(self, client, joints, positions, duration):
        if joints == ARM_JOINTS:
            ok, why = check_limits(positions)
            if not ok:
                raise StepFailed('plan', f'关节超限，拒绝执行：{why}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        t0 = time.time()
        while not fut.done():
            time.sleep(0.02)
            if time.time() - t0 > 10:
                raise StepFailed('comm', '发送轨迹目标超时（通信异常）')
        gh = fut.result()
        if not gh.accepted:
            raise StepFailed('controller', '控制器拒绝轨迹目标')
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done():
            time.sleep(0.02)
            if time.time() - t0 > duration * 10 + 60:
                raise StepFailed('controller', '轨迹执行超时')
        res = rf.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise StepFailed('controller', f'轨迹执行失败 error_code={res.error_code} {res.error_string}')

    def move_arm(self, q, duration=None):
        self._send_traj(self.arm_cli, ARM_JOINTS, list(q), duration or self.move_dur)

    def move_gripper(self, pos):
        self._send_traj(self.hand_cli, [GRIPPER_JOINT], [pos], self.grip_dur)

    # ---------- 规划 ----------
    # 【讲解】按网格编号/料盒编号查固定点并逆解全部路径点；任一点无解 → plan 失败（不可达），动作不执行
    # 【讲解】按网格编号/区域编号查预规划表并拼成完整路径点；不可达 → plan 失败（目标不可达），动作不执行
    def plan(self, grid_id, zone_id, slot):
        if grid_id not in self.cells:
            raise StepFailed('plan', f'网格编号 {grid_id} 不存在')
        if zone_id not in self.zones:
            raise StepFailed('plan', f'区域编号 {zone_id} 不存在')
        slot = slot % len(self.zones[zone_id]['slots'])
        cell_wp = self.cell_plans.get(grid_id)
        if cell_wp is None:
            try:
                cell_wp = self._taught_cell(grid_id) if self.use_taught else plan_cell(self.geo, grid_id, self._plan_params())
            except Unreachable as e:
                cell_wp = e
            self.cell_plans[grid_id] = cell_wp
        if isinstance(cell_wp, Exception):
            raise StepFailed('plan', str(cell_wp))
        slot_wp = self.slot_plans.get((zone_id, slot))
        if slot_wp is None:
            try:
                slot_wp = self._taught_slot(zone_id, slot) if self.use_taught else plan_slot(self.geo, zone_id, slot, self._plan_params())
            except Unreachable as e:
                slot_wp = e
            self.slot_plans[(zone_id, slot)] = slot_wp
        if isinstance(slot_wp, Exception):
            raise StepFailed('plan', str(slot_wp))
        return compose(cell_wp, slot_wp), self.cells[grid_id]

    # ---------- 夹取校验 ----------
    def _check_grasp(self, cube_name, z_before):
        if self.grasp_check == 'sim_pose':
            if cube_name is None:
                return False, '该网格位置没有物体（真值），夹空'
            p = self.truth.get(cube_name)
            if p is None:
                return False, f'读不到 {cube_name} 的位姿'
            dz = float(p[2] - z_before)
            return (dz >= self.lift_dz), f'{cube_name} 抬升 {dz * 1000:.1f} mm'
        if self.grasp_check == 'gripper':
            pos = self.joint_q.get(GRIPPER_JOINT)
            if pos is None:
                return False, '读不到夹爪关节'
            empty = abs(pos - self.grip_close) < self.grip_empty_tol
            return (not empty), f'夹爪读数 {pos:.3f}（目标 {self.grip_close:.2f}）'
        return True, '未启用夹取校验'

    # ---------- 动作执行 ----------
    def _execute(self, gh):
        self.busy = True
        t0 = time.time()
        result = PickPlace.Result()
        goal = gh.request
        try:
            if goal.command in ('park', 'home'):
                self._run_steps(gh, [('张开夹爪', lambda: self.move_gripper(self.grip_open)),
                                     ('回零', lambda: self.move_arm(self.home_q))])
            else:
                self._pick_place(gh, goal)
            result.success, result.failed_stage, result.message = True, '', '完成'
            gh.succeed()
        except StepFailed as e:
            result.success, result.failed_stage, result.message = False, e.stage, e.message
            note = self._safe_return()
            result.message = e.message + note
            self.status(f'动作失败[{e.stage}]：{result.message}')
            gh.abort()
        except Exception as e:  # noqa: BLE001
            result.success, result.failed_stage, result.message = False, 'internal', repr(e)
            self.get_logger().error(f'内部异常：{e!r}')
            self._safe_return()
            gh.abort()
        finally:
            result.duration = float(time.time() - t0)
            self.busy = False
        return result

    # 【讲解】失败后的安全返回：手里若还夹着物体（夹取校验已通过），先回到放置点上方再松开，物体落在分类区而不是掉在半路；
    # 没夹着就直接张爪、回零。返回一段说明文字附在结果里
    def _safe_return(self):
        note = ''
        if self.holding is not None:
            # 依次尝试：放置点上方 → 取物格过渡点（刚从那里过来，肯定能到）→ 都不行才原地松爪
            released = False
            for name, q in (('放置点上方', self.holding), ('取物格过渡点', self.holding_fallback)):
                if q is None:
                    continue
                try:
                    self.move_arm(q, self.move_dur * 0.6)
                    note = f'；已带着物体回到{name}后释放'
                    released = True
                    break
                except StepFailed as e:
                    note += f'；带物体回{name}失败（{e.message}）'
            if not released:
                note += '，原地释放'
        try:
            self.move_gripper(self.grip_open)
        except StepFailed:
            pass
        self.holding = None
        self.holding_fallback = None
        try:
            self.move_arm(self.home_q)
        except StepFailed as e:
            self.get_logger().error(f'返回安全位置失败：{e.message}')
            note += '；回零失败'
        return note

    def _run_steps(self, gh, steps):
        for i, (name, fn) in enumerate(steps, 1):
            fb = PickPlace.Feedback()
            fb.stage, fb.step, fb.total_steps = name, i, len(steps)
            gh.publish_feedback(fb)
            self.status(f'  [{i}/{len(steps)}] {name}')
            fn()

    # 【讲解】一次完整的"取物 → 分类放置 → 返回"。夹取校验失败时松爪、回扫描姿态并返回 grasp_verify
    def _pick_place(self, gh, goal):
        self.status(f'PickPlace：网格 {goal.grid_id}（{goal.object_class}）→ 区域 {goal.zone_id} 槽位 {goal.slot}')
        wp, (cx, cy) = self.plan(goal.grid_id, goal.zone_id, goal.slot)
        cube, z0 = None, None
        if self.truth is not None:
            cube = self.truth.nearest((cx, cy), max_dist=0.03)
            p = self.truth.get(cube) if cube else None
            z0 = float(p[2]) if p is not None else None
        inject = False
        if goal.grid_id in self.inject_grids:
            n = self.inject_count.get(goal.grid_id, 0)
            if n < self.inject_times:
                inject = True
                self.inject_count[goal.grid_id] = n + 1
                self.get_logger().warn(f'[异常测试] 网格 {goal.grid_id} 第 {n + 1} 次夹取注入失败（不合爪，模拟打滑）')

        def grasp():
            if inject:
                return
            try:
                self.move_gripper(self.grip_close)
            except StepFailed as e:
                # 真机驱动会读夹爪值：夹空/被挡时合爪动作直接报失败，这里归到"夹取失败"让任务节点重试
                if e.stage == 'controller':
                    raise StepFailed('grasp_verify', f'合爪失败（夹空/夹偏）：{e.message}')
                raise

        def verify():
            ok, why = self._check_grasp(cube, z0)
            self.status(f'  夹取校验：{why}')
            if not ok:
                raise StepFailed('grasp_verify', f'夹取失败（夹空）：{why}')
            self.holding = wp['above_zone']
            self.holding_fallback = wp['transit']

        def release():
            self.move_gripper(self.grip_open)
            self.holding = None

        steps = [
            ('张开夹爪', lambda: self.move_gripper(self.grip_open)),
            ('到达取物点上方', lambda: self.move_arm(wp['above_cell'])),
            ('下降', lambda: self.move_arm(wp['pick'], self.move_dur * 0.6)),
            ('夹取', grasp),
            ('抬升', lambda: self.move_arm(wp['above_cell'], self.move_dur * 0.6)),
            ('夹取校验', verify),
            ('抬到过渡高度', lambda: self.move_arm(wp['transit'], self.move_dur * 0.5)),
            ('转向放置区域', lambda: self.move_arm(wp['carry'])),
            ('移动到放置点上方', lambda: self.move_arm(wp['above_zone'], self.move_dur * 0.8)),
            ('下降到放置高度', lambda: self.move_arm(wp['place'], self.move_dur * 0.6)),
            ('释放', release),
            ('抬升离开', lambda: self.move_arm(wp['above_zone'], self.move_dur * 0.6)),
            ('抬回过渡高度', lambda: self.move_arm(wp['carry'], self.move_dur * 0.6)),
            ('转回正前方', lambda: self.move_arm(wp['back'])),
            ('回零', lambda: self.move_arm(self.home_q)),
        ]
        self._run_steps(gh, steps)

    def destroy_node(self):
        with self.traj_lock:
            self.traj_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PickPlaceServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
