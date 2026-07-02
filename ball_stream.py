#!/usr/bin/env python3
"""
Ball detector stream - based on benchmark.py Hailo API structure.
Replaces the original model with best.hef (YOLO26n, single class "ball").
Uses picamera2 for CSI camera, streams MJPEG over HTTP.
"""

import argparse
import collections
import json
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from hailo_platform import HEF, VDevice, HailoSchedulingAlgorithm, FormatType
from hailo_platform.pyhailort.pyhailort import FormatOrder

# ---------------------------------------------------------------------------
# YOLO26n detection constants
# ---------------------------------------------------------------------------
INPUT_H, INPUT_W = 960, 1280
STRIDES = (8, 16, 32)
GRID_SIZES = tuple((INPUT_H // s, INPUT_W // s) for s in STRIDES)
CONF_DEFAULT = 0.1


def _build_grid(gh, gw, stride):
    ys = np.arange(gh, dtype=np.float32) + 0.5
    xs = np.arange(gw, dtype=np.float32) + 0.5
    xs, ys = np.meshgrid(xs, ys)
    return xs * stride, ys * stride


GRIDS = {
    (gh, gw): _build_grid(gh, gw, STRIDES[i])
    for i, (gh, gw) in enumerate(GRID_SIZES)
}


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def decode_head(outputs: dict, conf_threshold: float):
    """Decode YOLO26n one2one head tensors into detection dicts."""
    # Build shape → slot name mapping (handle both with/without batch dim)
    shape_to_slot = {}
    for (gh, gw) in GRID_SIZES:
        shape_to_slot[(gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(gh, gw, 4)] = f"reg_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 4)] = f"reg_{gh}x{gw}"

    slots = {}
    for name, arr in outputs.items():
        slot = shape_to_slot.get(tuple(arr.shape))
        if slot:
            slots[slot] = arr

    results = []
    for i, (gh, gw) in enumerate(GRID_SIZES):
        cls_key = f"cls_{gh}x{gw}"
        reg_key = f"reg_{gh}x{gw}"
        if cls_key not in slots or reg_key not in slots:
            continue

        conf = _sigmoid(slots[cls_key].reshape(-1))
        mask = conf > conf_threshold
        if not mask.any():
            continue

        stride = STRIDES[i]
        cx_flat = GRIDS[(gh, gw)][0].reshape(-1)[mask]
        cy_flat = GRIDS[(gh, gw)][1].reshape(-1)[mask]
        conf_sel = conf[mask]

        reg = slots[reg_key].reshape(-1, 4)[mask]
        x1 = np.clip(cx_flat - reg[:, 0] * stride, 0, INPUT_W)
        y1 = np.clip(cy_flat - reg[:, 1] * stride, 0, INPUT_H)
        x2 = np.clip(cx_flat + reg[:, 2] * stride, 0, INPUT_W)
        y2 = np.clip(cy_flat + reg[:, 3] * stride, 0, INPUT_H)

        for j in range(len(conf_sel)):
            results.append({
                "conf": float(conf_sel[j]),
                "x1": float(x1[j]), "y1": float(y1[j]),
                "x2": float(x2[j]), "y2": float(y2[j]),
            })

    return results


def draw_detections(image: np.ndarray, dets: list) -> np.ndarray:
    img = image.copy()
    for d in dets:
        x1, y1, x2, y2 = int(d["x1"]), int(d["y1"]), int(d["x2"]), int(d["y2"])
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"ball {d['conf']:.2f}", (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img


def read_cpu_temp() -> float | None:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Hailo detector - copied from benchmark.py, adapted for best.hef outputs
# ---------------------------------------------------------------------------
class BallDetector:
    """
    Wraps best.hef using the same create_infer_model API as benchmark.py.
    """

    def __init__(self, hef_path: str):
        self.hef_path = hef_path
        self._vdevice = None
        self._hef = None
        self._infer_model = None
        self._config_ctx = None
        self._configured_model = None
        self._input_name = None
        self._output_names: list[str] = []
        self._output_shapes: dict[str, tuple] = {}
        self._output_types: dict[str, str] = {}
        self._model_input_shape: tuple = ()

    def init(self):
        vdev_params = VDevice.create_params()
        vdev_params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self._vdevice = VDevice(vdev_params)

        self._hef = HEF(self.hef_path)
        self._infer_model = self._vdevice.create_infer_model(self.hef_path)
        self._infer_model.set_batch_size(1)

        # Request FLOAT32 outputs so HailoRT DEQUANTIZES the INT8 tensors back to
        # real logits. Reading the HEF-native UINT8 and applying sigmoid to raw
        # quantized integers is meaningless (gave balls=0). This matches the proven
        # common.py reference (OutputVStreamParams.make(..., FormatType.FLOAT32)).
        output_infos = self._hef.get_output_vstream_infos()
        for info in output_infos:
            self._output_names.append(info.name)
            self._output_shapes[info.name] = tuple(info.shape)
            self._output_types[info.name] = "float32"
            self._infer_model.output(info.name).set_format_type(FormatType.FLOAT32)

        self._config_ctx = self._infer_model.configure()
        self._configured_model = self._config_ctx.__enter__()

        input_info = self._hef.get_input_vstream_infos()[0]
        self._input_name = input_info.name
        self._model_input_shape = tuple(input_info.shape)

        print(f"[init] HEF: {Path(self.hef_path).name}")
        print(f"[init] Input '{self._input_name}': {self._model_input_shape}")
        for n in self._output_names:
            print(f"[init] Output '{n}': {self._output_shapes[n]}  type={self._output_types[n]}")

    def preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        """
        Resize frame to model input size, convert BGR→RGB, return float32.
        Our model shape is (H, W, C) without batch dim: shape[0]=H, shape[1]=W.
        """
        model_h = self._model_input_shape[0]
        model_w = self._model_input_shape[1]
        resized = cv2.resize(frame_bgr, (model_w, model_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return rgb  # uint8, normalization /255 is baked into the HEF

    def infer(self, input_tensor: np.ndarray) -> dict:
        """
        Run sync inference.
        run_async expects a LIST of bindings, not a single Bindings object.
        """
        output_buffers = {
            name: np.empty(self._output_shapes[name],
                           dtype=getattr(np, self._output_types[name].lower()))
            for name in self._output_names
        }
        binding = self._configured_model.create_bindings(output_buffers=output_buffers)
        binding.input().set_buffer(input_tensor)
        self._configured_model.wait_for_async_ready(timeout_ms=10000)
        job = self._configured_model.run_async([binding])  # must be a list
        job.wait(10000)
        return {name: binding.output(name).get_buffer() for name in self._output_names}

    def cleanup(self):
        if self._config_ctx:
            self._config_ctx.__exit__(None, None, None)
        if self._vdevice:
            del self._vdevice
        print("[cleanup] Hailo released.")


# ---------------------------------------------------------------------------
# Camera + detection stream thread
# ---------------------------------------------------------------------------
class CameraDetectionStream:
    def __init__(self, detector: BallDetector, conf_threshold=CONF_DEFAULT,
                 jpeg_quality=80):
        self.detector = detector
        self.conf_threshold = conf_threshold
        self.jpeg_quality = jpeg_quality

        self.latest_jpeg: bytes | None = None
        self.frame_id = 0
        self.fps = 0.0
        self.last_dt = 0.0
        self.balls_last = 0
        self.error: str | None = None
        self.running = False
        self._frame_times: collections.deque = collections.deque(maxlen=30)
        self.condition = threading.Condition()
        self.thread: threading.Thread | None = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, name="cam-detect", daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        with self.condition:
            self.condition.notify_all()
        if self.thread:
            self.thread.join(timeout=5)

    def wait_for_frame(self, last_id: int, timeout: float = 5.0):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame_id != last_id or not self.running or self.error,
                timeout=timeout,
            )
            return self.frame_id, self.latest_jpeg, self.error

    def _set_error(self, msg: str):
        self.error = msg
        print(f"[ERROR] {msg}", flush=True)
        with self.condition:
            self.condition.notify_all()

    def _run(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            self._set_error("picamera2 not installed")
            return

        cam = Picamera2(0)
        cam.configure(cam.create_video_configuration(
            main={"size": (INPUT_W, INPUT_H), "format": "RGB888"},
            buffer_count=4,
        ))
        cam.set_controls({"AeEnable": True, "AwbEnable": True, "AeExposureMode": 0})
        cam.start()
        time.sleep(1.5)  # let AWB/AEC converge

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        print(f"[INFO] Camera started ({INPUT_W}x{INPUT_H}), conf={self.conf_threshold}", flush=True)

        try:
            while self.running:
                frame_bgr = cam.capture_array()  # RGB888 → numpy BGR array (V4L2 convention)

                t0 = time.perf_counter()
                input_tensor = self.detector.preprocess(frame_bgr)
                outputs = self.detector.infer(input_tensor)
                dets = decode_head(outputs, self.conf_threshold)
                dt = time.perf_counter() - t0

                rendered = draw_detections(frame_bgr, dets)
                ok, buf = cv2.imencode(".jpg", rendered, encode_params)
                if not ok:
                    continue

                now = time.perf_counter()
                self._frame_times.append(now)
                if len(self._frame_times) >= 2:
                    self.fps = (len(self._frame_times) - 1) / (
                        self._frame_times[-1] - self._frame_times[0]
                    )
                self.last_dt = dt
                self.balls_last = len(dets)

                with self.condition:
                    self.latest_jpeg = buf.tobytes()
                    self.frame_id += 1
                    self.condition.notify_all()

                print(
                    f"[frame {self.frame_id}] balls={len(dets):4d}  "
                    f"dt={dt * 1000:.1f}ms  fps={self.fps:.1f}",
                    flush=True,
                )
        except Exception as exc:
            self._set_error(str(exc))
        finally:
            cam.stop()


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
_stream: CameraDetectionStream | None = None

_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Ball Detector</title>
<style>
*{box-sizing:border-box}
body{background:#111;color:#eee;font-family:monospace;margin:0;padding:12px}
h1{margin:0 0 10px;font-size:1.2em}
#stats{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:10px}
.s{background:#1e1e1e;border:1px solid #333;padding:6px 14px;border-radius:6px;min-width:90px}
.s .v{font-size:1.5em;font-weight:bold;color:#4f4}
.s .l{font-size:.75em;color:#888}
img{max-width:100%;border-radius:4px}
</style></head><body>
<h1>Ball Detector</h1>
<div id="stats">
  <div class="s"><div class="v" id="fps">-</div><div class="l">FPS</div></div>
  <div class="s"><div class="v" id="ms">-</div><div class="l">Inference ms</div></div>
  <div class="s"><div class="v" id="balls">-</div><div class="l">Balls</div></div>
  <div class="s"><div class="v" id="temp">-</div><div class="l">CPU °C</div></div>
</div>
<img src="/stream">
<script>
setInterval(()=>{
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent=d.fps??'-';
    document.getElementById('ms').textContent=d.inference_ms??'-';
    document.getElementById('balls').textContent=d.balls_last??'-';
    document.getElementById('temp').textContent=d.cpu_temp_c??'-';
  }).catch(()=>{});
},1000);
</script></body></html>"""


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = _PAGE.encode()
            self._send(200, "text/html; charset=utf-8", body)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_id = -1
            try:
                while True:
                    fid, jpeg, err = _stream.wait_for_frame(last_id)
                    if err or not _stream.running or jpeg is None:
                        break
                    last_id = fid
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == "/api/status":
            body = json.dumps({
                "fps": round(_stream.fps, 1),
                "inference_ms": round(_stream.last_dt * 1000, 1),
                "balls_last": _stream.balls_last,
                "frame_id": _stream.frame_id,
                "cpu_temp_c": read_cpu_temp(),
                "error": _stream.error,
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
    global _stream

    parser = argparse.ArgumentParser(description="Ball detector MJPEG stream")
    parser.add_argument("--model", default="/root/best.hef")
    parser.add_argument("--conf-threshold", type=float, default=CONF_DEFAULT)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    args = parser.parse_args()

    detector = BallDetector(args.model)
    detector.init()

    _stream = CameraDetectionStream(
        detector=detector,
        conf_threshold=args.conf_threshold,
        jpeg_quality=args.jpeg_quality,
    )
    _stream.start()

    server = ThreadingHTTPServer((args.host, args.port), StreamHandler)
    print(f"[server] http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _stream.stop()
        detector.cleanup()


if __name__ == "__main__":
    main()
