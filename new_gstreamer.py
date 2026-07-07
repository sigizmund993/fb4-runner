#!/usr/bin/env python3
"""
Zero-copy ball detector: GStreamer + Hailo AI Hat + MJPEG HTTP stream.

Pipeline:
  libcamerasrc → videoconvert → videoscale(1280x960)
    → hailonet(best.hef) → hailofilter(libyolo26_post.so)
    → appsink

Python callback:
  - reads detections from HailoROI (zero-copy)
  - applies NMS
  - maps buffer READ-only → np.frombuffer (zero-copy) → draws boxes
  - encodes JPEG → pushes to MJPEG HTTP server
"""

import argparse
import collections
import signal
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import cv2

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GLib", "2.0")
from gi.repository import Gst, GLib

import hailo

# ---------------------------------------------------------------------------
# Model constants
# ---------------------------------------------------------------------------
INPUT_H, INPUT_W = 960, 1280
CONF_DEFAULT     = 0.25
IOU_DEFAULT      = 0.45


# ---------------------------------------------------------------------------
# NMS
# ---------------------------------------------------------------------------
def nms(dets: list, iou_threshold: float) -> list:
    if not dets:
        return []
    dets = sorted(dets, key=lambda d: d["conf"], reverse=True)
    keep = []
    while dets:
        best = dets.pop(0)
        keep.append(best)
        rest = []
        for d in dets:
            ix1  = max(best["x1"], d["x1"])
            iy1  = max(best["y1"], d["y1"])
            ix2  = min(best["x2"], d["x2"])
            iy2  = min(best["y2"], d["y2"])
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            area_a = (best["x2"] - best["x1"]) * (best["y2"] - best["y1"])
            area_b = (d["x2"]    - d["x1"])    * (d["y2"]    - d["y1"])
            union  = area_a + area_b - inter
            if union <= 0 or inter / union < iou_threshold:
                rest.append(d)
        dets = rest
    return keep


# ---------------------------------------------------------------------------
# Read detections from HailoROI (written by libyolo26_post.so)
# ---------------------------------------------------------------------------
def read_detections(roi, conf_threshold: float, iou_threshold: float) -> list:
    results = []
    for det in hailo.get_hailo_detections(roi):
        conf = det.get_confidence()
        if conf < conf_threshold:
            continue
        bbox = det.get_bbox()
        x1 = bbox.xmin()               * INPUT_W
        y1 = bbox.ymin()               * INPUT_H
        x2 = (bbox.xmin() + bbox.width())  * INPUT_W
        y2 = (bbox.ymin() + bbox.height()) * INPUT_H
        if x2 <= x1 or y2 <= y1:
            continue
        results.append({
            "conf":  float(conf),
            "x1":    float(x1),  "y1": float(y1),
            "x2":    float(x2),  "y2": float(y2),
            "label": det.get_label(),
        })
    return nms(results, iou_threshold)


# ---------------------------------------------------------------------------
# Draw detections onto RGB frame (in-place)
# ---------------------------------------------------------------------------
def draw_detections(frame_rgb: np.ndarray, dets: list) -> np.ndarray:
    """Draw center crosshair of best detection, return BGR for JPEG."""
    img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    if not dets:
        return img
    b  = max(dets, key=lambda d: d["conf"])
    cx = int((b["x1"] + b["x2"]) * 0.5)
    cy = int((b["y1"] + b["y2"]) * 0.5)
    r  = 12
    cv2.line(img, (cx - r, cy), (cx + r, cy), (0, 255, 0), 2)
    cv2.line(img, (cx, cy - r), (cx, cy + r), (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), r, (0, 255, 0), 2)
    cv2.putText(img, f"{b['conf']:.2f}", (cx + r + 4, cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return img


# ---------------------------------------------------------------------------
# Shared state between GStreamer thread and HTTP server
# ---------------------------------------------------------------------------
class FrameState:
    def __init__(self):
        self.lock      = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.jpeg: bytes | None = None
        self.frame_id  = 0
        self.fps       = 0.0
        self.dt_ms     = 0.0
        self.n_dets    = 0

    def push(self, jpeg: bytes, fps: float, dt_ms: float, n_dets: int):
        with self.condition:
            self.jpeg     = jpeg
            self.frame_id += 1
            self.fps      = fps
            self.dt_ms    = dt_ms
            self.n_dets   = n_dets
            self.condition.notify_all()

    def wait(self, last_id: int, timeout: float = 5.0):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame_id != last_id or self.jpeg is None,
                timeout=timeout,
            )
            return self.frame_id, self.jpeg


# ---------------------------------------------------------------------------
# GStreamer pipeline
# ---------------------------------------------------------------------------
PIPELINE_TMPL = """\
libcamerasrc ! \
videoconvert ! \
video/x-raw,format=RGB,width=1296,height=972,framerate=30/1 ! \
videoscale ! \
video/x-raw,format=RGB,width={w},height={h} ! \
queue leaky=downstream max-size-buffers=2 ! \
hailonet \
    hef-path={hef} \
    batch-size=1 \
    scheduling-algorithm=1 ! \
queue leaky=downstream max-size-buffers=2 ! \
hailofilter \
    so-path={so} \
    qos=false ! \
queue leaky=downstream max-size-buffers=2 ! \
appsink name=sink emit-signals=true sync=false drop=true max-buffers=2\
"""


class BallDetectorGst:
    def __init__(self, hef_path: str, so_path: str,
                 conf_threshold: float, iou_threshold: float,
                 jpeg_quality: int, state: FrameState):
        self.hef_path       = hef_path
        self.so_path        = so_path
        self.conf_threshold = conf_threshold
        self.iou_threshold  = iou_threshold
        self.jpeg_quality   = jpeg_quality
        self.state          = state

        self._pipeline: Gst.Pipeline | None = None
        self._loop: GLib.MainLoop | None    = None
        self._frame_times: collections.deque = collections.deque(maxlen=30)
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    def build(self):
        Gst.init(None)
        desc = PIPELINE_TMPL.format(
            w=INPUT_W, h=INPUT_H,
            hef=self.hef_path, so=self.so_path,
        )
        print(f"[pipeline]\n{desc}\n", flush=True)
        self._pipeline = Gst.parse_launch(desc)

        sink = self._pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_new_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::eos",   self._on_eos)

    def run(self):
        self._loop = GLib.MainLoop()
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("[ERROR] Pipeline failed to start", flush=True)
            sys.exit(1)
        print("[INFO] Pipeline running. Ctrl+C to stop.", flush=True)
        try:
            self._loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        print("[INFO] Stopping pipeline...", flush=True)
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
        if self._loop and self._loop.is_running():
            self._loop.quit()

    def _on_new_sample(self, sink) -> Gst.FlowReturn:
        t0 = time.perf_counter()

        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()

        # --- Read detections (zero-copy via HailoROI meta) ---
        try:
            roi  = hailo.get_roi_from_buffer(buf)
            dets = read_detections(roi, self.conf_threshold, self.iou_threshold)
        except Exception as e:
            print(f"[WARN] roi: {e}", flush=True)
            dets = []

        # --- Map buffer read-only → numpy view (zero-copy) ---
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            # Frame is RGB, shape (H, W, 3)
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape(INPUT_H, INPUT_W, 3)
            bgr   = draw_detections(frame, dets)   # copy + draw + BGR convert
        finally:
            buf.unmap(mapinfo)                      # release buffer back to pool

        # --- Encode JPEG ---
        ok2, jpg_buf = cv2.imencode(".jpg", bgr, self._encode_params)
        if not ok2:
            return Gst.FlowReturn.OK

        dt_ms = (time.perf_counter() - t0) * 1000
        now   = time.perf_counter()
        self._frame_times.append(now)
        fps = (
            (len(self._frame_times) - 1) /
            (self._frame_times[-1] - self._frame_times[0])
            if len(self._frame_times) >= 2 else 0.0
        )

        self.state.push(jpg_buf.tobytes(), fps, dt_ms, len(dets))

        # --- Console output ---
        n = len(dets)
        line = (
            f"[frame {self.state.frame_id:06d}] "
            f"balls={n:3d}  dt={dt_ms:6.2f}ms  fps={fps:5.1f}"
        )
        if dets:
            b = max(dets, key=lambda d: d["conf"])
            line += (
                f"  best [{b['label']}] {b['conf']:.3f} "
                f"({b['x1']:.0f},{b['y1']:.0f},{b['x2']:.0f},{b['y2']:.0f})"
            )
        print(line, flush=True)

        return Gst.FlowReturn.OK

    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[GST ERROR] {err.message}\n  {dbg}", flush=True)
        self.stop()

    def _on_eos(self, bus, msg):
        print("[GST EOS]", flush=True)
        self.stop()


# ---------------------------------------------------------------------------
# MJPEG HTTP server
# ---------------------------------------------------------------------------
_PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Ball Detector</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: monospace; padding: 12px; }
    h1 { font-size: 1.1em; margin-bottom: 10px; }
    #stats { display: flex; gap: 10px; margin-bottom: 10px; flex-wrap: wrap; }
    .s { background: #1e1e1e; border: 1px solid #333; padding: 6px 14px; border-radius: 6px; }
    .v { font-size: 1.5em; font-weight: bold; color: #4f4; }
    .l { font-size: .75em; color: #888; }
    img { max-width: 100%; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>Ball Detector</h1>
  <div id="stats">
    <div class="s"><div class="v" id="fps">-</div><div class="l">FPS</div></div>
    <div class="s"><div class="v" id="ms">-</div><div class="l">ms</div></div>
    <div class="s"><div class="v" id="balls">-</div><div class="l">Balls</div></div>
  </div>
  <img src="/stream">
  <script>
    setInterval(() => {
      fetch('/api/status').then(r => r.json()).then(d => {
        document.getElementById('fps').textContent   = d.fps   ?? '-';
        document.getElementById('ms').textContent    = d.dt_ms ?? '-';
        document.getElementById('balls').textContent = d.balls ?? '-';
      }).catch(() => {});
    }, 500);
  </script>
</body>
</html>"""

_state: FrameState | None = None


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = _PAGE.encode()
            self._send(200, "text/html; charset=utf-8", body)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_id = -1
            try:
                while True:
                    fid, jpeg = _state.wait(last_id)
                    if jpeg is None:
                        break
                    last_id = fid
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpeg + b"\r\n"
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == "/api/status":
            import json
            body = json.dumps({
                "fps":   round(_state.fps, 1),
                "dt_ms": round(_state.dt_ms, 1),
                "balls": _state.n_dets,
            }).encode()
            self._send(200, "application/json", body)

        else:
            self._send(404, "text/plain", b"not found")

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    global _state

    parser = argparse.ArgumentParser(description="Zero-copy ball detector + MJPEG stream")
    parser.add_argument("--model",          default="/root/best.hef")
    parser.add_argument("--so",             default="/root/rpi_fb4-runner/libyolo26_post.so")
    parser.add_argument("--conf-threshold", type=float, default=CONF_DEFAULT)
    parser.add_argument("--iou-threshold",  type=float, default=IOU_DEFAULT)
    parser.add_argument("--jpeg-quality",   type=int,   default=75)
    parser.add_argument("--host",           default="0.0.0.0")
    parser.add_argument("--port",           type=int,   default=8000)
    args = parser.parse_args()

    _state = FrameState()

    detector = BallDetectorGst(
        hef_path=args.model,
        so_path=args.so,
        conf_threshold=args.conf_threshold,
        iou_threshold=args.iou_threshold,
        jpeg_quality=args.jpeg_quality,
        state=_state,
    )

    # Start HTTP server in background thread
    server = ThreadingHTTPServer((args.host, args.port), MJPEGHandler)
    srv_thread = threading.Thread(target=server.serve_forever, daemon=True)
    srv_thread.start()
    print(f"[HTTP] http://{args.host}:{args.port}/", flush=True)

    signal.signal(signal.SIGTERM, lambda *_: detector.stop())

    detector.build()
    detector.run()

    server.shutdown()


if __name__ == "__main__":
    main()