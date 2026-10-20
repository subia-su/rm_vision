#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""armor_visualizer —— 实时显示 YOLO 检测框 + 装甲板颜色/类别/置信度。

订阅 /image_raw 与 /detector/yolo_boxes（armor_detector 发布），在图像上叠加：
  - OBB 旋转框（红方=红框，蓝方=蓝框）
  - 类别（R_1/B_G 等）、编号、置信度、颜色
  - 画面角落统计：红/蓝装甲板数量、FPS

用法:
  python3 armor_visualizer.py                 # 网页模式，http://<ip>:8081/
  python3 armor_visualizer.py --mode window   # OpenCV 窗口（需显示器）
  python3 armor_visualizer.py --save DIR      # 每帧保存标注图到 DIR
"""
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from auto_aim_interfaces.msg import YoloBoxes
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)

INDEX_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>armor visualizer</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:monospace}
.bar{padding:8px;font-size:14px}
img{display:block;max-width:100%}
</style></head><body>
<div class="bar">YOLO 检测框可视化 · 红框=RED 蓝框=BLUE · FPS <b id="fps">--</b></div>
<img id="stream" src="/stream.mjpg">
<script>
setInterval(async()=>{try{const d=await(await fetch('/status',{cache:'no-store'})).json();
document.getElementById('fps').textContent=d.fps.toFixed(1);}catch(e){}},1000);
</script>
</body></html>
"""


class ArmorVisualizer(Node):
    """维护最新图像与检测框，提供标注渲染。"""

    def __init__(self):
        super().__init__("armor_visualizer")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None           # 最新 bgr8 原始帧
        self.boxes = None           # 最新 YoloBoxes
        self._frames = 0
        self._t0 = time.time()
        self.fps = 0.0
        self.jpeg = None            # 渲染好的标注 JPEG（供网页）

        self.create_subscription(Image, "/image_raw", self.on_image, QOS)
        self.create_subscription(YoloBoxes, "/detector/yolo_boxes", self.on_boxes, QOS)
        self.get_logger().info("armor_visualizer started, watching /image_raw + /detector/yolo_boxes")

    def on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.frame = img
            self._frames += 1
            now = time.time()
            if now - self._t0 >= 1.0:
                self.fps = self._frames / (now - self._t0)
                self._frames = 0
                self._t0 = now

    def on_boxes(self, msg):
        with self.lock:
            self.boxes = msg

    # ---- 渲染 ----
    def render(self):
        """把最新帧 + 检测框合成标注图（返回 bgr 图）。"""
        with self.lock:
            if self.frame is None:
                return None
            img = self.frame.copy()
            boxes = self.boxes
        if boxes is None:
            return img

        n_red = n_blue = 0
        for box in boxes.boxes:
            if len(box.corners) < 4:
                continue
            pts = [(int(p.x), int(p.y)) for p in box.corners[:4]]
            is_red = box.color == "RED"
            if is_red:
                n_red += 1
            else:
                n_blue += 1
            # BGR：红 (0,0,255)，蓝 (255,0,0)
            color = (0, 0, 255) if is_red else (255, 0, 0)
            cv2.polylines(img, [np.array(pts, dtype=np.int32)], True, color, 2)
            # 标签：类别 编号 颜色 置信度
            label = "%s %s %s %.2f" % (
                box.class_name, box.number, box.color, box.confidence)
            tx, ty = pts[1][0], max(0, pts[1][1] - 8)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(img, (tx, ty - th - 4), (tx + tw + 4, ty + 2), color, -1)
            cv2.putText(img, label, (tx + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            # 中心十字
            cx = int(sum(p[0] for p in pts) / 4)
            cy = int(sum(p[1] for p in pts) / 4)
            cv2.drawMarker(img, (cx, cy), color, cv2.MARKER_CROSS, 12, 1)

        # 画面信息栏
        h, w = img.shape[:2]
        info = "RED=%d BLUE=%d boxes=%d" % (n_red, n_blue, len(boxes.boxes))
        cv2.rectangle(img, (0, 0), (w, 30), (40, 40, 40), -1)
        cv2.putText(img, info, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 1, cv2.LINE_AA)
        return img

    def render_jpeg(self, quality=85):
        img = self.render()
        if img is None:
            return None
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes() if ok else None

    def status(self):
        with self.lock:
            n = 0 if self.boxes is None else len(self.boxes.boxes)
            return {"fps": round(self.fps, 1), "boxes": n}


class Handler(BaseHTTPRequestHandler):
    viz = None  # type: ArmorVisualizer

    def log_message(self, *args):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send(INDEX_HTML.encode(), "text/html; charset=utf-8")
        elif self.path.startswith("/status"):
            self._send(json.dumps(self.viz.status()).encode(), "application/json")
        elif self.path.startswith("/snapshot.jpg"):
            self._send(self.viz.render_jpeg() or b"", "image/jpeg")
        elif self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                jpg = self.viz.render_jpeg()
                if jpg:
                    try:
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
                time.sleep(1.0 / 30)
        else:
            self.send_response(404)
            self.end_headers()


def run_web(viz, host, port):
    Handler.viz = viz
    httpd = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(">>> 浏览器打开: http://<jetson-ip>:%d/" % port, flush=True)
    print(">>> 单帧:       curl -o snap.jpg http://127.0.0.1:%d/snapshot.jpg" % port, flush=True)
    try:
        rclpy.spin(viz)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()


def run_window(viz):
    cv2.namedWindow("armor visualizer (Esc to quit)", cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            rclpy.spin_once(viz, timeout_sec=0.01)
            img = viz.render()
            if img is not None:
                st = viz.status()
                cv2.putText(img, "%.1f fps" % st["fps"], (10, img.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("armor visualizer (Esc to quit)", img)
            if cv2.waitKey(1) == 27:
                break
    finally:
        cv2.destroyAllWindows()


def run_save(viz, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    print(">>> 每帧保存标注图到 %s" % save_dir, flush=True)
    while rclpy.ok():
        rclpy.spin_once(viz, timeout_sec=0.01)
        img = viz.render()
        if img is not None:
            ts = time.strftime("%H%M%S", time.localtime())
            ms = int(time.time() * 1000) % 1000
            path = os.path.join(save_dir, "vis_%s_%03d.jpg" % (ts, ms))
            cv2.imwrite(path, img)
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="YOLO 装甲板检测框实时可视化")
    parser.add_argument("--mode", choices=["web", "window", "save"], default="web")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--save", default="/tmp/armor_vis")
    args = parser.parse_args()

    rclpy.init()
    viz = ArmorVisualizer()

    if args.mode == "window":
        run_window(viz)
    elif args.mode == "save":
        run_save(viz, args.save)
    else:
        run_web(viz, "0.0.0.0", args.port)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
