#!/usr/bin/env python3
"""Live camera view in a web browser, with a focus (sharpness) score.

Works over SSH: the Pi serves the picture, your laptop's browser shows it.
Nothing here talks to the Pixhawk.

Two sources:
  ROS (default)   reads /camera/image_raw, so it runs NEXT TO the stack
                  (start perception.launch.py first, in terminal A).
  --device        opens /dev/video0 directly with OpenCV. Only when NO ROS
                  camera node is running (the camera can only be opened once).

Then open  http://<pi-ip>:8080  (or http://viruta.local:8080) on your laptop.

Focus: turn the lens slowly. "sharp" goes up as the picture gets crisper;
"best" remembers the highest value seen. Aim at something with fine detail
(printed text, a checkerboard) 2-3 m away, the distance people will be at.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class Latest:
    """Newest frame (BGR) shared between the camera side and the web side."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.count = 0
        self.best = 0.0
        self.t_last = time.monotonic()
        self.fps = 0.0

    def put(self, bgr):
        h, w = bgr.shape[:2]
        # sharpness = variance of the Laplacian over the centre half of the picture
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        crop = gray[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        sharp = float(cv2.Laplacian(crop, cv2.CV_64F).var())
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now
        fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if dt > 0 else self.fps

        out = bgr.copy()
        cv2.rectangle(out, (w // 4, h // 4), (3 * w // 4, 3 * h // 4), (0, 255, 255), 1)
        with self.lock:
            self.best = max(self.best, sharp)
            best = self.best
        colour = (0, 255, 0) if sharp >= 0.85 * best else (0, 165, 255)
        cv2.putText(out, f"sharp {sharp:7.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
        cv2.putText(out, f"best  {best:7.1f}", (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(out, f"{w}x{h}  {fps:4.1f} fps", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        ok, jpg = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.jpeg = jpg.tobytes()
                self.count += 1
                self.fps = fps

    def reset_best(self):
        with self.lock:
            self.best = 0.0

    def get(self):
        with self.lock:
            return self.jpeg, self.count


PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Drone camera</title>
<style>body{background:#111;color:#ddd;font-family:sans-serif;text-align:center;margin:12px}
img{max-width:100%;border:1px solid #444}a{color:#6cf}</style></head><body>
<img src="/stream"><p>Turn the lens slowly; stop where <b>sharp</b> is highest (green = within 15% of best).
<a href="/reset">reset best</a></p></body></html>"""


def make_handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/reset":
                latest.reset_best()
                self.send_response(303)
                self.send_header("Location", "/")
                self.end_headers()
                return
            if self.path != "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(PAGE)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seen = -1
            try:
                while True:
                    jpg, n = latest.get()
                    if jpg is None or n == seen:
                        time.sleep(0.01)
                        continue
                    seen = n
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser tab closed

    return Handler


def run_device(latest, device, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"cannot open {device} (is a ROS camera node still running?)")
    print(f"reading {device} directly")
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        latest.put(frame)


def run_ros(latest, topic):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("camera_view")
    # same QoS as the rest of the stack (RELIABLE, newest frame only)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

    def on_image(msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img
        elif msg.encoding == "mono8":
            img = buf.reshape(msg.height, msg.step)[:, : msg.width]
            bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            node.get_logger().warn(f"encoding {msg.encoding} not handled", throttle_duration_sec=5.0)
            return
        latest.put(bgr)

    node.create_subscription(Image, topic, on_image, qos)
    print(f"reading ROS topic {topic}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--device", help="read this device directly instead of ROS, e.g. /dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    a = ap.parse_args()

    latest = Latest()
    server = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(latest))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"open http://<pi-ip>:{a.port}  (Ctrl-C to stop)")
    try:
        if a.device:
            run_device(latest, a.device, a.width, a.height)
        else:
            run_ros(latest, a.topic)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
