#!/usr/bin/env python3
# 【讲解】识别节点（仿真阶段用颜色分割代替 YOLO）。
# 输入：sensor_msgs/Image（仿真俯视相机，经 ros_gz_bridge 桥接）。
# 输出：vision_msgs/Detection2DArray —— 每个物体一个 Detection2D：类别 class_id、检测框 bbox、置信度 score（验收要求的三要素）。
#       另发一路画了框的调试图 /detections/image，可选写成 mp4 作为演示视频素材。
# 算法：BGR→HSV → 取"高饱和度且够亮"的像素做前景（桌面是灰色、机械臂是白/黑，都不饱和）→ 连通域 → 每个连通域按色相
#       投票判红/绿/蓝；投票纯度不够（例如黄色方块）→ 类别 "unknown"、低置信度，交给任务节点按"未识别物体"跳过。
# 讲解要点：真机阶段把这个节点换成 YOLO 节点即可，话题和消息类型不变（识别接口 = Detection2DArray）。
"""颜色分割识别节点：Image → Detection2DArray（+ 调试图）。"""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

# 类别 → OpenCV 色相区间列表（H ∈ [0,179]）。红色跨 0 点，给两个区间
DEFAULT_HUE = {
    'red': [(0, 8), (168, 179)],
    'green': [(40, 85)],
    'blue': [(95, 130)],
}
DRAW_COLOR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0), 'unknown': (0, 200, 255)}


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        for name, default in [
            ('image_topic', '/camera/image_raw'), ('detections_topic', '/detections'),
            ('debug_image_topic', '/detections/image'),
            ('min_saturation', 110), ('min_value', 60),
            ('min_area', 120), ('max_area', 8000),
            ('min_purity', 0.6),          # 主色相占比低于此 → unknown
            ('roi', [0, 0, 0, 0]),        # [u0,v0,u1,v1] 像素 ROI，全 0 = 整幅
            ('record_video', ''),         # 非空 = 把调试图写成 mp4
            ('record_fps', 10.0),
            ('publish_rate_limit', 0.0),  # 0 = 每帧都发
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.s_min, self.v_min = int(g('min_saturation')), int(g('min_value'))
        self.min_area, self.max_area = int(g('min_area')), int(g('max_area'))
        self.min_purity = float(g('min_purity'))
        self.roi = [int(v) for v in g('roi')]
        self.hue = DEFAULT_HUE
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, g('detections_topic'), 10)
        self.dbg_pub = self.create_publisher(Image, g('debug_image_topic'), 5)
        self.create_subscription(Image, g('image_topic'), self._on_image, 5)
        self.writer = None
        self.record_path = os.path.expanduser(g('record_video'))
        self.record_fps = float(g('record_fps'))
        self.frames = 0
        self.t_last_log = 0.0
        self.get_logger().info(f"识别节点启动：订阅 {g('image_topic')} → 发布 {g('detections_topic')}")

    # 【讲解】给一个连通域的像素色相做投票：返回 (最佳类别, 纯度)
    def _classify(self, hues):
        best, best_frac = 'unknown', 0.0
        n = max(1, hues.size)
        for cls, ranges in self.hue.items():
            m = np.zeros(hues.shape, dtype=bool)
            for lo, hi in ranges:
                m |= (hues >= lo) & (hues <= hi)
            frac = float(m.sum()) / n
            if frac > best_frac:
                best, best_frac = cls, frac
        if best_frac < self.min_purity:
            return 'unknown', best_frac
        return best, best_frac

    # 【讲解】每来一帧图像：分割 → 连通域 → 分类 → 发 Detection2DArray 与调试图
    def _on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'图像转换失败: {e}')
            return
        h, w = bgr.shape[:2]
        u0, v0, u1, v1 = self.roi if any(self.roi) else (0, 0, w, h)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        fg = (hsv[:, :, 1] >= self.s_min) & (hsv[:, :, 2] >= self.v_min)
        roi_mask = np.zeros((h, w), dtype=bool)
        roi_mask[v0:v1, u0:u1] = True
        fg &= roi_mask
        fg8 = fg.astype(np.uint8)
        fg8 = cv2.morphologyEx(fg8, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, labels, stats, cents = cv2.connectedComponentsWithStats(fg8, connectivity=8)

        out = Detection2DArray()
        out.header = msg.header
        dbg = bgr.copy()
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if area < self.min_area or area > self.max_area:
                continue
            comp = labels == i
            cls, purity = self._classify(hsv[:, :, 0][comp])
            fill = float(area) / float(max(1, bw * bh))
            score = 0.7 * purity + 0.3 * fill if cls != 'unknown' else min(0.49, 0.5 * purity + 0.2 * fill)
            det = Detection2D()
            det.header = msg.header
            det.id = f'{cls}_{i}'
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float(cents[i][0])
            det.bbox.center.position.y = float(cents[i][1])
            det.bbox.size_x, det.bbox.size_y = float(bw), float(bh)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
            c = DRAW_COLOR.get(cls, (255, 255, 255))
            cv2.rectangle(dbg, (int(x), int(y)), (int(x + bw), int(y + bh)), c, 2)
            cv2.putText(dbg, f'{cls} {score:.2f}', (int(x), max(12, int(y) - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv2.LINE_AA)
        if any(self.roi):
            cv2.rectangle(dbg, (u0, v0), (u1, v1), (200, 200, 200), 1)
        cv2.putText(dbg, f'detections: {len(out.detections)}', (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.pub.publish(out)
        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        dbg_msg.header = msg.header
        self.dbg_pub.publish(dbg_msg)
        self._record(dbg)
        self.frames += 1
        now = time.time()
        if now - self.t_last_log > 5.0:
            self.t_last_log = now
            summary = ', '.join(f"{d.results[0].hypothesis.class_id}:{d.results[0].hypothesis.score:.2f}" for d in out.detections)
            self.get_logger().info(f'帧 {self.frames}：{len(out.detections)} 个目标 [{summary}]')

    # 【讲解】可选：把调试图连续写成 mp4（相机视角演示视频）
    def _record(self, frame):
        if not self.record_path:
            return
        if self.writer is None:
            os.makedirs(os.path.dirname(self.record_path) or '.', exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.record_path, fourcc, self.record_fps, (frame.shape[1], frame.shape[0]))
            self.get_logger().info(f'开始录制相机视角视频 → {self.record_path}')
        self.writer.write(frame)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
