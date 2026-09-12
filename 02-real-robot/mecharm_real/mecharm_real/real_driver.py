#!/usr/bin/env python3
# 【讲解】真机驱动节点，跑在 Jetson。它是"翻译官"：
# 上面对任务节点提供与仿真 ros2_control **一模一样**的三个接口（两个 FollowJointTrajectory 动作服务器 + /joint_states），
# 下面通过 TCP 把指令转给臂内树莓派上的 arm_server.py，由它调 pymycobot 驱动舵机。
# 因此任务节点 ros_node.py 一行不改就能从仿真切到真机（验收要求）。
# 结构：
#   ① 参数：臂内服务地址、速度上限、到位容差、关节方向/偏置、夹爪开合角与读值判定阈值。
#   ② 两条 TCP 连接：motion 走运动（会阻塞到到位），state 走 10 Hz 状态轮询与急停，互不等待。
#   ③ _publish_states：读角度 → 发 /joint_states。
#   ④ _exec_arm：收到六关节轨迹目标 → 查 URDF 限位 → 度数与速度换算 → 臂内 goto → 按残差判成败。
#   ⑤ _exec_hand：夹爪目标 → 开/合 → 读回值判定有没有夹住（20–60 算夹住），不对就松开重夹一次。
#   ⑥ /soft_stop 软件急停：只 stop 不松舵机，防止手臂坠落。
# 讲解要点：任何异常都以"动作失败"返回给任务节点，任务节点会记失败并回安全位；成败判据是读回的角度/夹爪值，不信指令返回码。
"""mechArm 270 真机驱动节点（Jetson 侧）。

对外提供与仿真阶段 ros2_control 完全相同的 ROS 2 接口，任务节点
mecharm_grasp/ros_node.py 一行不改即可复用（验收要求：切真机只改设备、通信、位置参数）：
  - /arm_controller/follow_joint_trajectory    FollowJointTrajectory 动作
  - /hand_controller/follow_joint_trajectory   FollowJointTrajectory 动作
  - /joint_states                              关节状态（默认 10Hz）
  - /soft_stop (std_msgs/Bool, data=true)      软件急停：立即停止并拒绝后续目标

底层不直接碰串口：通过 TCP 调用臂内树莓派上的 arm_server.py，
后者原样复用 02-real-robot/scripts/arm_common.py（MechArm270 类、自定限位、直发角度）。
两条连接：motion 走运动（会阻塞到到位），state 走状态轮询与急停，互不等待。

安全：
  - max_speed 限制速度百分比，初次运行低速
  - 每个目标先查 URDF 限位，臂内再按实机限位查一遍
  - 通信异常 / 到位残差过大 → 动作返回失败，任务节点走"返回安全位置"分支
  - 急停只 stop 不松舵机，防止手臂坠落
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from mecharm_real.arm_client import ArmClient, ArmError
from mecharm_real.kinematics import (ARM_JOINTS, GRIPPER_JOINT, JOINT_LIMITS,
                                     deg_to_rad, rad_to_deg, speed_percent, gripper_state)


# 【讲解】驱动节点类，接口名与仿真控制器保持一致
class MechArmRealDriver(Node):
    def __init__(self):
        super().__init__('mecharm_real_driver')
        for name, default in [
            ('arm_host', '10.42.0.89'), ('arm_port', 9001),
            ('max_speed', 20),            # 速度百分比上限（1-100）
            ('state_rate', 10.0),
            ('goal_tol_deg', 3.0),        # 到位判定：各关节残差上限
            ('joint_signs', [1.0] * 6),
            ('joint_offsets_deg', [0.0] * 6),
            ('gripper_open_rad', 0.0), ('gripper_close_rad', -0.44),
            # 合爪读值判定：夹住 25mm 方块≈43–45，空夹到底≈4，被挡住≈70+
            ('grip_hold_min', 20), ('grip_hold_max', 60), ('open_min', 80),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.max_speed = int(g('max_speed'))
        self.tol = float(g('goal_tol_deg'))
        self.signs, self.offsets = list(g('joint_signs')), list(g('joint_offsets_deg'))
        self.g_open, self.g_close = float(g('gripper_open_rad')), float(g('gripper_close_rad'))
        self.hold_min, self.hold_max, self.open_min = int(g('grip_hold_min')), int(g('grip_hold_max')), int(g('open_min'))

        host, port = str(g('arm_host')), int(g('arm_port'))
        self.motion = ArmClient(host, port, timeout=30.0)
        self.state = ArmClient(host, port, timeout=5.0)
        info = self.state.ping()   # 连不上直接抛错退出，比起来再失败清楚
        self.stopped = False
        self.busy = threading.Lock()
        self.last_deg = None
        self.last_gripper_rad = self.g_open

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Bool, '/soft_stop', self._on_soft_stop, 10, callback_group=cb)
        self.arm_srv = ActionServer(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                                    self._exec_arm, callback_group=cb)
        self.hand_srv = ActionServer(self, FollowJointTrajectory, '/hand_controller/follow_joint_trajectory',
                                     self._exec_hand, callback_group=cb)
        self.create_timer(1.0 / float(g('state_rate')), self._publish_states, callback_group=cb)
        self.get_logger().info(f"真机驱动就绪：臂内服务 {host}:{port}{'（假臂）' if info.get('fake') else ''}，"
                               f"低速模式 max_speed={self.max_speed}")

    # ---------- 状态 ----------
    # 【讲解】定时器回调：经 state 连接读六关节角度（度）→ 换成弧度 → 发布 /joint_states，任务节点靠它写轨迹日志
    def _publish_states(self):
        try:
            degs = self.state.get_angles().get('angles')
        except ArmError as e:
            self.get_logger().warn(f'读角度失败: {e}', throttle_duration_sec=5.0)
            return
        if not degs or len(degs) != 6:
            return
        self.last_deg = degs
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS + [GRIPPER_JOINT]
        js.position = deg_to_rad(degs, self.signs, self.offsets) + [self.last_gripper_rad]
        self.js_pub.publish(js)

    # ---------- 急停 ----------
    # 【讲解】收到 /soft_stop true：置急停标志并让臂 stop，后续目标一律拒绝；false 解除
    def _on_soft_stop(self, msg):
        if msg.data:
            self.stopped = True
            try:
                self.state.stop()
            except ArmError as e:
                self.get_logger().error(f'急停指令发送失败: {e}')
            self.get_logger().error('软件急停触发：已停止运动，拒绝后续目标（恢复需发布 /soft_stop data:false）')
        else:
            self.stopped = False
            self.get_logger().info('软件急停解除')

    # ---------- 动作 ----------
    # 【讲解】把成功/失败包装成动作结果；失败时 error_string 写原因，任务节点会打到 errors.log
    def _result(self, gh, ok, err=''):
        res = FollowJointTrajectory.Result()
        if ok:
            res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            gh.succeed()
        else:
            res.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            res.error_string = err
            self.get_logger().error(f'轨迹执行失败: {err}')
            gh.abort()
        return res

    @staticmethod
    # 【讲解】轨迹点的 time_from_start 转成秒，最短 0.5 s
    def _duration(pt):
        return max(0.5, pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)

    # 【讲解】手臂动作：校验关节名与限位 → 弧度转角度 → 由"角度差/时长"算速度百分比并封顶 → 臂内 goto 阻塞到到位 → 残差 > 容差判失败
    def _exec_arm(self, gh):
        traj = gh.request.trajectory
        if self.stopped:
            return self._result(gh, False, '处于急停状态')
        if list(traj.joint_names) != ARM_JOINTS or not traj.points:
            return self._result(gh, False, f'非法目标: {list(traj.joint_names)}')
        pt = traj.points[-1]
        q = [float(v) for v in pt.positions]
        for (lo, hi), v, name in zip(JOINT_LIMITS, q, ARM_JOINTS):
            if not (lo - 1e-6 <= v <= hi + 1e-6):
                return self._result(gh, False, f'关节 {name} 目标 {v:.3f} 超 URDF 限位')
        duration = self._duration(pt)
        target = [round(v, 2) for v in rad_to_deg(q, self.signs, self.offsets)]
        cur = self.last_deg or target
        delta = max(abs(t - c) for t, c in zip(target, cur))
        speed = speed_percent(delta, duration, self.max_speed)
        self.get_logger().info(f'-> 目标 {target} 速度 {speed}%')
        with self.busy:
            try:
                resp = self.motion.goto(target, speed, timeout=duration * 3 + 5, tol=1.5)
            except ArmError as e:
                return self._result(gh, False, f'臂内通信异常: {e}')
        if self.stopped:
            return self._result(gh, False, '执行中被急停')
        err = resp.get('err') or []
        self.last_deg = resp.get('angles') or self.last_deg
        if not err or max(err) > self.tol:
            return self._result(gh, False, f'到位残差 {err} 超过 {self.tol}°（通信或负载异常）')
        return self._result(gh, True)

    # 【讲解】夹爪动作：目标弧度离开/合哪个近就发哪个 → 臂内读回值 → 合爪不在 20–60 就松开重夹一次，仍不行判失败；张开不到 80 再张一次
    def _exec_hand(self, gh):
        traj = gh.request.trajectory
        if self.stopped:
            return self._result(gh, False, '处于急停状态')
        if list(traj.joint_names) != [GRIPPER_JOINT] or not traj.points:
            return self._result(gh, False, f'非法目标: {list(traj.joint_names)}')
        pt = traj.points[-1]
        rad = float(pt.positions[0])
        state = gripper_state(rad, self.g_open, self.g_close)
        t0 = time.time()
        with self.busy:
            try:
                resp = self.motion.gripper(state)   # 臂内会等 2s 读回，没动会重发
            except ArmError as e:
                return self._result(gh, False, f'夹爪通信异常: {e}')
        if resp.get('reached') is False:
            return self._result(gh, False, f"夹爪未动作（指令 {'合' if state else '开'}，"
                                           f"读值 {resp.get('before')}→{resp.get('value')}，共发 {resp.get('tries')} 次）")
        if resp.get('tries', 1) > 1:
            self.get_logger().warn(f"夹爪指令重发 {resp['tries'] - 1} 次才动作（读值 {resp.get('before')}→{resp.get('value')}）")
        v = resp.get('value')
        if isinstance(v, (int, float)):
            if state == 1 and not (self.hold_min <= v <= self.hold_max):
                # 合爪读值不在"夹住物体"区间：被挡住/夹偏(>max) 或 夹空(<min)。松开再合一次，还不行判失败
                self.get_logger().warn(f'合爪读值 {v} 不在 {self.hold_min}–{self.hold_max}，松开重夹一次')
                try:
                    with self.busy:
                        self.motion.gripper(0)
                        resp = self.motion.gripper(1)
                except ArmError as e:
                    return self._result(gh, False, f'夹爪通信异常: {e}')
                v = resp.get('value')
                if not (isinstance(v, (int, float)) and self.hold_min <= v <= self.hold_max):
                    why = '夹空（物体不在取物点）' if isinstance(v, (int, float)) and v < self.hold_min else '被挡住或夹偏'
                    return self._result(gh, False, f'夹取失败：合爪读值 {v}，{why}')
            elif state == 0 and v < self.open_min:
                # 张不全：手指还顶着物体/桌面，再张一次，仍不全只警告（物体多半已放下）
                try:
                    with self.busy:
                        resp = self.motion.gripper(0)
                except ArmError as e:
                    return self._result(gh, False, f'夹爪通信异常: {e}')
                v = resp.get('value')
                if isinstance(v, (int, float)) and v < self.open_min:
                    self.get_logger().warn(f'夹爪只张开到 {v}（正常≈89）：手指可能顶着桌面或物体，抬升时可能拖动物体')
        remain = self._duration(pt) - (time.time() - t0)
        if remain > 0:
            time.sleep(remain)
        self.last_gripper_rad = rad
        return self._result(gh, True)


# 【讲解】入口：起节点、多线程执行器；Ctrl+C 时先让臂 stop
def main():
    rclpy.init()
    node = MechArmRealDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        try:
            node.state.stop()
        except Exception:
            pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
