#!/usr/bin/env python3
"""rm_vision 相机实时监控程序（独立工具，只订阅不发布，不影响视觉管线）。

订阅 /image_raw，实时显示海康相机画面。

两种模式：
  --mode web     （默认）起一个网页，浏览器打开 http://<jetson-ip>:8080
                 —— Jetson 无显示器时用这个
  --mode window  原生 OpenCV 窗口（需要接显示器 / X11）

用法：
  source /opt/ros/foxy/setup.bash
  python3 camera_monitor.py                 # 网页模式，端口 8080
  python3 camera_monitor.py --port 8090
  python3 camera_monitor.py --mode window   # 有显示器时
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

INDEX_HTML = """<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>相机实时监控</title>
<style>
  body { margin:0; background:#111; color:#ddd; font-family:Consolas,monospace; }
  .bar { padding:8px 12px; font-size:13px; color:#aaa; }
  .bar b { color:#ffd93d; }
  img { display:block; width:100%; max-width:1280px; background:#000; }
</style>
</head>
<body>
  <div class="bar">相机实时画面 · 分辨率 <span id="res">--</span> · FPS <b id="fps">--</b></div>
  <img id="cam" src="/stream.mjpg" alt="camera">
<script>
async function loop() {
  try {
    const d = await (await fetch('/status', {cache:'no-store'})).json();
    document.getElementById('fps').textContent = d.fps.toFixed(1);
    document.getElementById('res').textContent = d.width + 'x' + d.height;
  } catch (e) {}
  setTimeout(loop, 1000);
}
loop();
</script>
</body>
</html>
"""


class CameraMonitor(Node):
    """订阅 /image_raw，维护最新缩略 JPEG 与帧统计。"""

    def __init__(self, max_width=960, quality=80):
        super().__init__("camera_monitor")
        self.bridge = CvBridge()
        self.max_width = max_width
        self.quality = quality
        self.lock = threading.Lock()

        self.jpeg = None
        self.width = 0
        self.height = 0
        self._frames = 0
        self._t0 = time.time()
        self.fps = 0.0

        self.create_subscription(Image, "/image_raw", self._on_image, SENSOR_QOS)
        self.get_logger().info("camera_monitor started, watching /image_raw")

    def _on_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        h, w = img.shape[:2]
        if w > self.max_width:
            scale = self.max_width / float(w)
            img = cv2.resize(img, (self.max_width, int(round(h * scale))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        with self.lock:
            self.jpeg = buf.tobytes()
            self.width = img.shape[1]
            self.height = img.shape[0]
            self._frames += 1
            now = time.time()
            if now - self._t0 >= 1.0:
                self.fps = self._frames / (now - self._t0)
                self._frames = 0
                self._t0 = now

    def get_jpeg(self):
        with self.lock:
            return self.jpeg

    def status(self):
        with self.lock:
            return {"fps": round(self.fps, 1), "width": self.width, "height": self.height}


class Handler(BaseHTTPRequestHandler):
    node = None  # type: CameraMonitor

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
        import json

        if self.path in ("/", "/index.html"):
            self._send(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path.startswith("/status"):
            self._send(json.dumps(self.node.status()).encode("utf-8"), "application/json")
        elif self.path.startswith("/snapshot.jpg"):
            self._send(self.node.get_jpeg() or b"", "image/jpeg")
        elif self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                jpg = self.node.get_jpeg()
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


def run_web(node, host, port):
    Handler.node = node
    httpd = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(">>> 浏览器打开:  http://<jetson-ip>:%d/" % port, flush=True)
    print(">>> 抓一帧:      curl -o snap.jpg http://127.0.0.1:%d/snapshot.jpg" % port, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()


def run_window(node):
    cv2.namedWindow("camera monitor  (Esc to quit)", cv2.WINDOW_NORMAL)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        img = node.get_jpeg()
        if img:
            frame = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
            st = node.status()
            cv2.putText(frame, "%.1f fps" % st["fps"], (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("camera monitor  (Esc to quit)", frame)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="rm_vision real-time camera monitor")
    ap.add_argument("--mode", choices=["web", "window"], default="web",
                    help="web: 浏览器查看（无显示器可用）; window: 原生 OpenCV 窗口")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=960, help="画面缩放宽")
    ap.add_argument("--quality", type=int, default=80, help="JPEG 质量 1-100")
    args = ap.parse_args()

    rclpy.init()
    node = CameraMonitor(max_width=args.width, quality=args.quality)
    try:
        if args.mode == "web":
            run_web(node, args.host, args.port)
        else:
            run_window(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
