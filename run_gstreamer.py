#!/usr/bin/env python3
import sys
import time
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib

try:
    import hailo
except ImportError:
    print("hailo not found, trying without ROI extraction")
    hailo = None

Gst.init(None)

HEF_PATH = "10_hours_brainrot.hef"
SO_PATH = "./libyolo26_post.so"
FRAME_W = 1280
FRAME_H = 960

def on_new_sample(sink):
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.ERROR

    buf = sample.get_buffer()
    ts = buf.get_pts()
    pts_s = ts / Gst.SECOND if ts != Gst.CLOCK_TIME_NONE else 0

    if hailo and buf:
        try:
            roi = hailo.get_roi_from_buffer(buf)
            if roi:
                dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
                for obj in dets:
                    bbox = obj.get_bbox()
                    cx = (bbox.xmin() + bbox.width() / 2.0) * FRAME_W
                    cy = (bbox.ymin() + bbox.height() / 2.0) * FRAME_H
                    conf = obj.get_confidence()
                    name = obj.get_label() or "ball"
                    print(
                        f"[{time.strftime('%H:%M:%S')}] "
                        f"{name} conf={conf:.2f} "
                        f"center=({cx:.0f},{cy:.0f}) "
                        f"bbox=({bbox.xmin()*FRAME_W:.0f},{bbox.ymin()*FRAME_H:.0f})-"
                        f"({(bbox.xmin()+bbox.width())*FRAME_W:.0f},{(bbox.ymin()+bbox.height())*FRAME_H:.0f})",
                        flush=True,
                    )
        except Exception as e:
            pass
    else:
        print(f"[{time.strftime('%H:%M:%S')}] frame pts={pts_s:.3f}s", flush=True)

    return Gst.FlowReturn.OK


pipeline_str = (
    f"libcamerasrc ! "
    f"video/x-raw,width={FRAME_W},height={FRAME_H},format=RGB,framerate=30/1 ! "
    f"videoconvert ! "
    f"hailonet hef-path={HEF_PATH} input-format-type=uint8 batch-size=1 ! "
    f"hailofilter so-path={SO_PATH} function-name=filter ! "
    f"appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
)

print(f"[init] Pipeline: hailonet({HEF_PATH}) + hailofilter({SO_PATH})", flush=True)
pipeline = Gst.parse_launch(pipeline_str)
sink = pipeline.get_by_name("sink")
sink.connect("new-sample", on_new_sample)

bus = pipeline.get_bus()
pipeline.set_state(Gst.State.PLAYING)
print("[init] Playing...", flush=True)

loop = GLib.MainLoop()

def check_bus():
    msg = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS, 0)
    if msg:
        if msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            print(f"[ERROR] {err}", flush=True)
        loop.quit()
        return False
    return True

GLib.timeout_add(100, check_bus)

try:
    loop.run()
except KeyboardInterrupt:
    pass
finally:
    pipeline.set_state(Gst.State.NULL)
    print("[done]", flush=True)
