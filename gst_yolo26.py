#!/usr/bin/env python3
"""
Ball detector stream using GStreamer + Hailo for zero-copy inference.
Uses hailopython for custom YOLO26n post-processing.
Includes graceful shutdown on KeyboardInterrupt / SIGTERM.
"""
import argparse
import time
import threading
import json
import os
import signal
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

Gst.init(None)

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

GRIDS = {(gh, gw): _build_grid(gh, gw, STRIDES[i]) for i, (gh, gw) in enumerate(GRID_SIZES)}
def _sigmoid(x): return 1.0 / (1.0 + np.exp(-x))

def decode_head(outputs, conf_threshold):
    shape_to_slot = {}
    for (gh, gw) in GRID_SIZES:
        shape_to_slot[(gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(gh, gw, 4)] = f"reg_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 4)] = f"reg_{gh}x{gw}"
    slots = {}
    for name, arr in outputs.items():
        slot = shape_to_slot.get(tuple(arr.shape))
        if slot: slots[slot] = arr
    results = []
    for i, (gh, gw) in enumerate(GRID_SIZES):
        cls_key = f"cls_{gh}x{gw}"
        reg_key = f"reg_{gh}x{gw}"
        if cls_key not in slots or reg_key not in slots: continue
        conf = _sigmoid(slots[cls_key].reshape(-1))
        mask = conf > conf_threshold
        if not mask.any(): continue
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
            results.append({"conf": float(conf_sel[j]), "x1": float(x1[j]), "y1": float(y1[j]), "x2": float(x2[j]), "y2": float(y2[j])})
    return results

# ---------------------------------------------------------------------------
# Hailopython Module Content (will be saved to /tmp)
# ---------------------------------------------------------------------------
HAILOPYTHON_MODULE = """
import hailo
from gsthailo import VideoFrame
from gi.repository import Gst
import numpy as np

INPUT_H, INPUT_W = 960, 1280
STRIDES = (8, 16, 32)
GRID_SIZES = tuple((INPUT_H // s, INPUT_W // s) for s in STRIDES)
CONF_DEFAULT = 0.1

def _build_grid(gh, gw, stride):
    ys = np.arange(gh, dtype=np.float32) + 0.5
    xs = np.arange(gw, dtype=np.float32) + 0.5
    xs, ys = np.meshgrid(xs, ys)
    return xs * stride, ys * stride

GRIDS = {(gh, gw): _build_grid(gh, gw, STRIDES[i]) for i, (gh, gw) in enumerate(GRID_SIZES)}
def _sigmoid(x): return 1.0 / (1.0 + np.exp(-x))

def decode_head(outputs, conf_threshold):
    shape_to_slot = {}
    for (gh, gw) in GRID_SIZES:
        shape_to_slot[(gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(gh, gw, 4)] = f"reg_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 4)] = f"reg_{gh}x{gw}"
    slots = {}
    for name, arr in outputs.items():
        slot = shape_to_slot.get(tuple(arr.shape))
        if slot: slots[slot] = arr
    results = []
    for i, (gh, gw) in enumerate(GRID_SIZES):
        cls_key = f"cls_{gh}x{gw}"
        reg_key = f"reg_{gh}x{gw}"
        if cls_key not in slots or reg_key not in slots: continue
        conf = _sigmoid(slots[cls_key].reshape(-1))
        mask = conf > conf_threshold
        if not mask.any(): continue
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
            results.append({"conf": float(conf_sel[j]), "x1": float(x1[j]), "y1": float(y1[j]), "x2": float(x2[j]), "y2": float(y2[j])})
    return results

def run(video_frame: VideoFrame):
    outputs = {}
    for tensor in video_frame.roi.get_tensors():
        # ZERO-COPY: получаем доступ к памяти тензора без копирования
        outputs[tensor.name()] = np.array(tensor, copy=False)
    
    dets = decode_head(outputs, CONF_DEFAULT)
    if dets:
        best_det = max(dets, key=lambda d: d['conf'])
        cx = (best_det['x1'] + best_det['x2']) / 2.0
        cy = (best_det['y1'] + best_det['y2']) / 2.0
        
        # Выводим центр в консоль
        print(f"[DETECTION] Center (Model 1280x960): ({cx:.1f}, {cy:.1f}) | Norm: ({cx/INPUT_W:.3f}, {cy/INPUT_H:.3f}) | Conf: {best_det['conf']:.3f}", flush=True)
        
        # Добавляем детект в ROI для отрисовки через hailooverlay
        norm_x1 = best_det['x1'] / INPUT_W
        norm_y1 = best_det['y1'] / INPUT_H
        norm_w = (best_det['x2'] - best_det['x1']) / INPUT_W
        norm_h = (best_det['y2'] - best_det['y1']) / INPUT_H
        bbox = hailo.HailoBBox(xmin=norm_x1, ymin=norm_y1, width=norm_w, height=norm_h)
        detection = hailo.HailoDetection(bbox=bbox, label='ball', confidence=best_det['conf'])
        video_frame.roi.add_object(detection)
        
    return Gst.FlowReturn.OK
"""

# ---------------------------------------------------------------------------
# GStreamer Pipeline Wrapper
# ---------------------------------------------------------------------------
class GStreamerStream:
    def __init__(self, hef_path):
        self.hef_path = hef_path
        self.loop = GLib.MainLoop()
        self.thread = None
        self.running = False
        
        self.fps = 0.0
        self.frame_id = 0
        self.latest_jpeg = None
        self.condition = threading.Condition()
        self.frame_times = []
        
        self.pipeline = None
        self.appsink = None
        self.module_path = "/tmp/hailo_yolo26n_postprocess.py"

    def start(self):
        self.running = True
        with open(self.module_path, "w") as f:
            f.write(HAILOPYTHON_MODULE)
            
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        if not self.running:
            return
        self.running = False
        print("\n[INFO] Stopping GStreamer pipeline...")
        
        # 1. Останавливаем пайплайн (освобождает камеру и NPU)
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            
        # 2. Останавливаем GLib MainLoop
        if self.loop and self.loop.is_running():
            self.loop.quit()
            
        # 3. Ждем завершения потока
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5)
            
        # 4. Чистим временный файл
        if os.path.exists(self.module_path):
            os.remove(self.module_path)
        print("[INFO] GStreamer stopped.")

    def wait_for_frame(self, last_id, timeout=5.0):
        with self.condition:
            self.condition.wait_for(lambda: self.frame_id != last_id or not self.running, timeout=timeout)
            return self.frame_id, self.latest_jpeg

    def _run(self):
        pipeline_str = (
            f"libcamerasrc ! video/x-raw,format=NV12 "
            f"! videoconvert ! video/x-raw,format=RGB "
            f"! queue leaky=no max-size-buffers=3 "
            f"! hailonet hef-path={self.hef_path} batch-size=1 "
            f"! queue leaky=no max-size-buffers=3 "
            f"! hailopython module={self.module_path} qos=false "
            f"! queue leaky=no max-size-buffers=3 "
            f"! hailooverlay ! videoconvert ! video/x-raw,format=RGB "
            f"! appsink name=sink emit-signals=true max-buffers=1 drop=true"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.on_new_sample)
        
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
        except Exception as e:
            print(f"[ERROR] GStreamer loop failed: {e}")
        finally:
            # На случай, если loop завершился сам (например, EOS), все равно чистим пайплайн
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)

    def on_new_sample(self, sink):
        if not self.running:
            return Gst.FlowReturn.ERROR
        
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            w = caps.get_structure(0).get_value('width')
            h = caps.get_structure(0).get_value('height')
            
            success, map = buf.map(Gst.MapFlags.READ)
            if success:
                # ZERO-COPY: создаем numpy array поверх памяти GStreamer буфера
                frame = np.ndarray(shape=(h, w, 3), buffer=map.data, dtype=np.uint8)
                ok, buf_jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                buf.unmap(map)
                
                if ok:
                    now = time.perf_counter()
                    self.frame_times.append(now)
                    if len(self.frame_times) > 30: self.frame_times.pop(0)
                    if len(self.frame_times) >= 2:
                        self.fps = (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])
                    
                    self.frame_id += 1
                    self.latest_jpeg = buf_jpeg.tobytes()
                    with self.condition:
                        self.condition.notify_all()
        return Gst.FlowReturn.OK

# ---------------------------------------------------------------------------
# HTTP Server
# ---------------------------------------------------------------------------
_stream: GStreamerStream | None = None
_server: ThreadingHTTPServer | None = None

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
<h1>Ball Detector (GStreamer Zero-Copy)</h1>
<div id="stats">
  <div class="s"><div class="v" id="fps">-</div><div class="l">FPS</div></div>
  <div class="s"><div class="v" id="balls">-</div><div class="l">Last Conf</div></div>
</div>
<img src="/stream">
<script>
setInterval(()=>{
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent=d.fps??'-';
    document.getElementById('balls').textContent=d.last_conf??'-';
  }).catch(()=>{});
},1000);
</script></body></html>"""

def read_cpu_temp() -> float | None:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None

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
                    fid, jpeg = _stream.wait_for_frame(last_id)
                    if not _stream.running or jpeg is None: break
                    last_id = fid
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif self.path == "/api/status":
            body = json.dumps({
                "fps": round(_stream.fps, 1),
                "cpu_temp_c": read_cpu_temp(),
                "last_conf": "See console for center coords"
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
# Entry point with Graceful Shutdown
# ---------------------------------------------------------------------------
def main():
    global _stream, _server
    
    parser = argparse.ArgumentParser(description="Ball detector MJPEG stream via GStreamer")
    parser.add_argument("--model", default="/root/best.hef")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    _stream = GStreamerStream(args.model)
    _stream.start()

    _server = ThreadingHTTPServer((args.host, args.port), StreamHandler)
    print(f"[server] http://{args.host}:{args.port}/", flush=True)
    print("[INFO] Look at the console for [DETECTION] center coordinates!", flush=True)
    print("[INFO] Press Ctrl+C to stop gracefully.", flush=True)

    # Обработчик сигналов для осторожной остановки
    def shutdown_handler(sig, frame):
        print(f"\n[INFO] Signal {sig} received, shutting down gracefully...")
        if _server:
            _server.server_close()  # Закрываем слушающий сокет, чтобы serve_forever() вышел
        if _stream:
            _stream.stop()          # Останавливаем GStreamer и чистим ресурсы
        print("[INFO] Shutdown complete. Goodbye!")
        sys.exit(0)

    # Регистрируем обработчики для Ctrl+C (SIGINT) и kill (SIGTERM)
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        _server.serve_forever()
    except KeyboardInterrupt:
        # Fallback на случай, если сигнал не перехватился (например, в некоторых IDE)
        shutdown_handler(signal.SIGINT, None)
    except Exception as e:
        print(f"[ERROR] Server crashed: {e}")
    finally:
        # Дополнительная страховка
        if _server:
            _server.server_close()
        if _stream:
            _stream.stop()

if __name__ == "__main__":
    main()