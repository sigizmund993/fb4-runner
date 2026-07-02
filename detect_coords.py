#!/usr/bin/env python3
import time
import sys
import numpy as np
from picamera2 import Picamera2
from hailo_platform import HEF, VDevice, HailoSchedulingAlgorithm, FormatType

INPUT_H, INPUT_W = 960, 1280
STRIDES = (8, 16, 32)
GRID_SIZES = ((INPUT_H // s, INPUT_W // s) for s in STRIDES)
CONF_THRESHOLD = 0.15
HEF_PATH = "/root/fb4-runner/10_hours_brainrot.hef"


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def decode_head(outputs, conf_threshold):
    shape_to_slot = {}
    for gh, gw in GRID_SIZES:
        shape_to_slot[(gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(gh, gw, 4)] = f"reg_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 1)] = f"cls_{gh}x{gw}"
        shape_to_slot[(1, gh, gw, 4)] = f"reg_{gh}x{gw}"
    slots = {}
    for name, arr in outputs.items():
        s = shape_to_slot.get(tuple(arr.shape))
        if s:
            slots[s] = arr
    results = []
    for i, (gh, gw) in enumerate(GRID_SIZES):
        cls_key = f"cls_{gh}x{gw}"
        reg_key = f"reg_{gh}x{gw}"
        if cls_key not in slots or reg_key not in slots:
            continue
        stride = STRIDES[i]
        ys = np.arange(gh, dtype=np.float32) + 0.5
        xs = np.arange(gw, dtype=np.float32) + 0.5
        xs, ys = np.meshgrid(xs, ys)
        cx_grid = (xs * stride).reshape(-1)
        cy_grid = (ys * stride).reshape(-1)
        conf = sigmoid(slots[cls_key].reshape(-1))
        mask = conf > conf_threshold
        if not mask.any():
            continue
        reg = slots[reg_key].reshape(-1, 4)[mask]
        x1 = np.clip(cx_grid[mask] - reg[:, 0] * stride, 0, INPUT_W)
        y1 = np.clip(cy_grid[mask] - reg[:, 1] * stride, 0, INPUT_H)
        x2 = np.clip(cx_grid[mask] + reg[:, 2] * stride, 0, INPUT_W)
        y2 = np.clip(cy_grid[mask] + reg[:, 3] * stride, 0, INPUT_H)
        for j in range(len(conf[mask])):
            results.append({
                "conf": float(conf[mask][j]),
                "x1": float(x1[j]), "y1": float(y1[j]),
                "x2": float(x2[j]), "y2": float(y2[j]),
            })
    return results


def main():
    hef = HEF(HEF_PATH)
    vdev = VDevice.create_params()
    vdev.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
    device = VDevice(vdev)
    infer_model = device.create_infer_model(HEF_PATH)
    infer_model.set_batch_size(1)
    output_names = []
    output_shapes = {}
    for info in hef.get_output_vstream_infos():
        output_names.append(info.name)
        output_shapes[info.name] = tuple(info.shape)
        infer_model.output(info.name).set_format_type(FormatType.FLOAT32)
    config_ctx = infer_model.configure()
    configured = config_ctx.__enter__()
    input_info = hef.get_input_vstream_infos()[0]
    model_h = input_info.shape[0]
    model_w = input_info.shape[1]

    cam = Picamera2(0)
    cam.configure(cam.create_video_configuration(
        main={"size": (INPUT_W, INPUT_H), "format": "RGB888"},
        buffer_count=4,
    ))
    cam.set_controls({"AeEnable": True, "AwbEnable": True, "AeExposureMode": 0})
    cam.start()
    time.sleep(1.5)

    print(f"[detect] Model ready. Streaming detections (conf>{CONF_THRESHOLD})...", flush=True)
    frame_id = 0
    t0 = time.perf_counter()
    try:
        while True:
            frame = cam.capture_array()
            resized = np.array(__import__("cv2").resize(
                frame, (model_w, model_h), interpolation=__import__("cv2").INTER_LINEAR
            ))
            rgb = __import__("cv2").cvtColor(resized, __import__("cv2").COLOR_BGR2RGB)
            output_buffers = {
                name: np.empty(output_shapes[name], dtype=np.float32)
                for name in output_names
            }
            binding = configured.create_bindings(output_buffers=output_buffers)
            binding.input().set_buffer(rgb)
            configured.wait_for_async_ready(timeout_ms=10000)
            job = configured.run_async([binding])
            job.wait(10000)
            outputs = {name: binding.output(name).get_buffer() for name in output_names}
            dets = decode_head(outputs, CONF_THRESHOLD)
            frame_id += 1
            dt = time.perf_counter() - t0
            t0 = time.perf_counter()
            if dets:
                best = max(dets, key=lambda d: d["conf"])
                cx = (best["x1"] + best["x2"]) / 2
                cy = (best["y1"] + best["y2"]) / 2
                print(
                    f"[frame {frame_id:5d}] dets={len(dets):2d}  "
                    f"best={best['conf']:.2f}  "
                    f"center=({cx:.0f},{cy:.0f})  "
                    f"box=({best['x1']:.0f},{best['y1']:.0f})-({best['x2']:.0f},{best['y2']:.0f})  "
                    f"infer={dt*1000:.1f}ms",
                    flush=True,
                )
            else:
                print(
                    f"[frame {frame_id:5d}] dets=0  infer={dt*1000:.1f}ms",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        config_ctx.__exit__(None, None, None)
        del device
        print("[detect] Stopped.", flush=True)


if __name__ == "__main__":
    main()
