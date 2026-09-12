#!/usr/bin/env python3
# 【讲解】YOLO 识别节点（真机用，接口与仿真的 color_detector 完全一致）：
# 订阅 sensor_msgs/Image → ultralytics 推理 → 发布 vision_msgs/Detection2DArray（类别、检测框、置信度）+ 画框的调试图。
# 类别名映射：模型里叫 red_cube/green_cube/blue_cube/yellow_cube，任务节点用 red/green/blue；默认自动去掉 "_cube" 后缀，
# 也可以用 class_map 参数显式指定（"模型名:任务名"）。映射后不在任务类别表里的（如 yellow）由任务节点按"未识别物体"跳过。
# 推理跟不上相机帧率时丢帧只处理最新一帧，保证检测结果不滞后。
"""YOLO 识别节点：Image → Detection2DArray（+ 调试图）。"""
import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

COLORS = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        for name, default in [
            ('model_path', '~/Desktop/yingjiehua/best.pt'), ('conf', 0.5), ('iou', 0.5), ('imgsz', 640),
            ('device', '0'),                     # '0' = GPU 0，'cpu' = CPU
            ('half', True),                      # FP16 推理（Jetson 上快 20-30%）
            ('debug_scale', 0.5),                # 调试图缩放（只影响看图，不影响检测）
            ('debug_every', 2),                  # 每 N 帧发一张调试图，减轻 DDS 负担
            ('class_map', ['']),                 # 例如 ['red_cube:red', 'green_cube:green']；空 = 自动去 _cube 后缀
            ('image_topic', '/camera/image_raw'), ('detections_topic', '/detections'),
            ('debug_image_topic', '/detections/image'),
            ('record_video', ''), ('record_fps', 10.0),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        from ultralytics import YOLO  # 延迟导入：仿真机器上可能没装
        path = os.path.expanduser(str(g('model_path')))
        self.model = YOLO(path)
        self.conf, self.iou, self.imgsz = float(g('conf')), float(g('iou')), int(g('imgsz'))
        self.device = str(g('device'))
        self.half = bool(g('half'))
        self.debug_scale = float(g('debug_scale'))
        self.debug_every = max(1, int(g('debug_every')))
        self.infer_ms = 0.0
        self.names = dict(self.model.names)
        cmap = {}
        for item in g('class_map'):
            if item and ':' in item:
                a, b = item.split(':', 1)
                cmap[a.strip()] = b.strip()
        self.class_map = {n: cmap.get(n, n.replace('_cube', '')) for n in self.names.values()}
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, g('detections_topic'), 10)
        self.dbg_pub = self.create_publisher(Image, g('debug_image_topic'), 5)
        self.create_subscription(Image, g('image_topic'), self._on_image, 1)
        self.latest = None
        self.lock = threading.Lock()
        self.writer, self.record_path, self.record_fps = None, os.path.expanduser(g('record_video')), float(g('record_fps'))
        self.frames, self.t_last_log = 0, 0.0
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f'YOLO 识别节点：模型 {path}，类别 {self.names} → {self.class_map}，device={self.device}')

    def _on_image(self, msg):
        with self.lock:
            self.latest = msg     # 只留最新一帧，推理慢时自然丢帧

    def _worker(self):
        while rclpy.ok():
            with self.lock:
                msg, self.latest = self.latest, None
            if msg is None:
                time.sleep(0.005)
                continue
            try:
                self._process(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'推理异常：{e!r}')
                time.sleep(0.5)

    def _process(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t0 = time.time()
        res = self.model.predict(bgr, conf=self.conf, iou=self.iou, imgsz=self.imgsz, device=self.device, half=self.half, verbose=False)[0]
        self.infer_ms = 0.9 * self.infer_ms + 0.1 * (time.time() - t0) * 1000
        out = Detection2DArray()
        out.header = msg.header
        dbg = bgr.copy()
        if res.boxes is not None:
            for i in range(len(res.boxes)):
                x1, y1, x2, y2 = [float(v) for v in res.boxes.xyxy[i].tolist()]
                score = float(res.boxes.conf[i])
                raw = self.names.get(int(res.boxes.cls[i]), str(int(res.boxes.cls[i])))
                cls = self.class_map.get(raw, raw)
                det = Detection2D()
                det.header = msg.header
                det.id = f'{cls}_{i}'
                det.bbox = BoundingBox2D()
                det.bbox.center.position.x = (x1 + x2) / 2
                det.bbox.center.position.y = (y1 + y2) / 2
                det.bbox.size_x, det.bbox.size_y = x2 - x1, y2 - y1
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls
                hyp.hypothesis.score = score
                det.results.append(hyp)
                out.detections.append(det)
                c = COLORS.get(cls, (0, 200, 255))
                cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
                cv2.putText(dbg, f'{cls} {score:.2f}', (int(x1), max(12, int(y1) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
        cv2.putText(dbg, f'detections: {len(out.detections)}', (8, dbg.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        self.pub.publish(out)
        if self.frames % self.debug_every == 0:
            small = cv2.resize(dbg, None, fx=self.debug_scale, fy=self.debug_scale) if self.debug_scale != 1.0 else dbg
            dm = self.bridge.cv2_to_imgmsg(small, encoding='bgr8')
            dm.header = msg.header
            self.dbg_pub.publish(dm)
        if self.record_path:
            if self.writer is None:
                os.makedirs(os.path.dirname(self.record_path) or '.', exist_ok=True)
                self.writer = cv2.VideoWriter(self.record_path, cv2.VideoWriter_fourcc(*'mp4v'), self.record_fps, (dbg.shape[1], dbg.shape[0]))
            self.writer.write(dbg)
        self.frames += 1
        if time.time() - self.t_last_log > 5.0:
            self.t_last_log = time.time()
            summary = ', '.join(f"{d.results[0].hypothesis.class_id}:{d.results[0].hypothesis.score:.2f}" for d in out.detections)
            self.get_logger().info(f'帧 {self.frames}：{len(out.detections)} 个目标 [{summary}]，推理 {self.infer_ms:.0f} ms')

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
