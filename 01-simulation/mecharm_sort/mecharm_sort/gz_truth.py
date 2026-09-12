#!/usr/bin/env python3
# 【讲解】仿真真值读取：ros_gz_bridge 把 Gazebo 的 /world/<w>/dynamic_pose/info（所有会动的实体位姿）
# 桥成 tf2_msgs/TFMessage，这里把它解析成 {模型名: (x,y,z)}。只用于仿真里的成功判定/夹取校验，真机没有这个话题。
"""Gazebo 位姿真值订阅（仿真专用）。"""
import threading

import numpy as np
from tf2_msgs.msg import TFMessage


class GzTruth:
    """订阅 /sim/poses（TFMessage），按名字缓存位姿。"""

    def __init__(self, node, topic='/sim/poses', prefix='cube_', callback_group=None):
        self.prefix = prefix
        self.poses = {}
        self.lock = threading.Lock()
        self.received = False
        node.create_subscription(TFMessage, topic, self._cb, 10, callback_group=callback_group)

    def _cb(self, msg):
        with self.lock:
            for t in msg.transforms:
                name = t.child_frame_id
                if self.prefix and not name.startswith(self.prefix):
                    continue
                p = t.transform.translation
                self.poses[name] = np.array([p.x, p.y, p.z])
            self.received = True

    def get(self, name):
        with self.lock:
            p = self.poses.get(name)
            return None if p is None else p.copy()

    def all(self):
        with self.lock:
            return {k: v.copy() for k, v in self.poses.items()}

    # 【讲解】找离某个平面坐标最近的物体（用于"这格里的方块叫什么名字"）
    def nearest(self, xy, max_dist=0.03):
        best, best_d = None, max_dist
        for name, p in self.all().items():
            d = float(np.linalg.norm(p[:2] - np.asarray(xy)[:2]))
            if d < best_d:
                best, best_d = name, d
        return best
