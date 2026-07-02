import os
import sys
import time
import struct
from multiprocessing import Process, shared_memory

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp, GLib

try:
    import hailo
except ImportError:
    print("[-] hailo not found")
    sys.exit(1)

Gst.init(None)

HEF_PATH = "10_hours_brainrot.hef"
# Вместо hailofilter используем hailodetection, который сам подгрузит нужный постпроцесс внутри hailonet
SHM_NAME = "hailo_ball_shm"
SHM_SIZE = 8
FRAME_WIDTH = 1280
FRAME_HEIGHT = 960

TARGET_CLASS_ID = 0
CONFIDENCE_THRESHOLD = 0.25

_inference_shm = None

def on_new_sample(sink):
    global _inference_shm
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.ERROR

    buffer = sample.get_buffer()
    if buffer and _inference_shm:
        center_x, center_y = -1, -1
        max_confidence = 0.0

        # Пытаемся забрать ROI
        roi = hailo.get_roi_from_buffer(buffer)
        if roi:
            detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
            for obj in detections:
                if obj.get_class_id() == TARGET_CLASS_ID:
                    conf = obj.get_confidence()
                    if conf >= CONFIDENCE_THRESHOLD and conf > max_confidence:
                        max_confidence = conf
                        bbox = obj.get_bbox()
                        
                        norm_center_x = bbox.xmin() + (bbox.width() / 2.0)
                        norm_center_y = bbox.ymin() + (bbox.height() / 2.0)
                        
                        center_x = int(norm_center_x * FRAME_WIDTH)
                        center_y = int(norm_center_y * FRAME_HEIGHT)

        try:
            _inference_shm.buf[:8] = struct.pack("ii", center_x, center_y)
        except Exception as e:
            pass

    return Gst.FlowReturn.OK


def run_hailo_inference(hef_path, shm_name):
    global _inference_shm
    
    try:
        _inference_shm = shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)
    except FileExistsError:
        _inference_shm = shared_memory.SharedMemory(name=shm_name)

    _inference_shm.buf[:8] = struct.pack("ii", -1, -1)

    # ИСХОДНЫЙ ИЗМЕНЕННЫЙ ПАЙПЛАЙН:
    # 1. Добавлен v4l2convert для жесткой гарантии NV12 формата от камеры
    # 2. Используем связку hailonet + hailodetection
    pipeline_str = (
        f"libcamerasrc ! "
        f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT} ! "
        f"v4l2convert ! "
        f"video/x-raw, format=NV12 ! "
        f"queue max-size-buffers=3 leaky=no ! "
        f"hailonet hef-path={hef_path} ! "
        f"hailodetection ! "
        f"queue max-size-buffers=3 ! "
        f"appsink name=yolosink emit-signals=true sync=false max-buffers=1 drop=true"
    )

    print("[AI-Process] Launching robust pipeline...")
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("yolosink")
    appsink.connect("new-sample", on_new_sample)

    # Добавим шину сообщений, чтобы увидеть, если GStreamer падает с ошибкой внутри
    bus = pipeline.get_bus()

    pipeline.set_state(Gst.State.PLAYING)
    print("[AI-Process] Pipeline is running.")

    loop = GLib.MainLoop()
    
    # Небольшой хак: проверяем ошибки в шине GStreamer параллельно
    def check_bus():
        message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS, 0)
        if message:
            if message.type == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                print(f"\n[GStreamer ERROR]: {err} \nDebug: {debug}\n")
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
        if _inference_shm:
            _inference_shm.close()
            try:
                _inference_shm.unlink()
            except FileNotFoundError:
                pass


def run_controller_worker(shm_name):
    while True:
        try:
            shm = shared_memory.SharedMemory(name=shm_name)
            break
        except FileNotFoundError:
            time.sleep(0.1)

    print("[Controller] Connected!")

    try:
        while True:
            raw_bytes = bytes(shm.buf[:8])
            x, y = struct.unpack("ii", raw_bytes)
            if x != -1 and y != -1:
                print(f"[{time.strftime('%H:%M:%S')}] МЯЧ НАЙДЕН! X={x}, Y={y}")
            else:
                print(f"[{time.strftime('%H:%M:%S')}] Ищу...")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        shm.close()


if __name__ == "__main__":
    if not os.path.exists(HEF_PATH):
        print(f"[-] {HEF_PATH} not found!")
        sys.exit(1)

    # Изменились аргументы (POST_PROC_SO больше не передаем)
    hailo_proc = Process(target=run_hailo_inference, args=(HEF_PATH, SHM_NAME))
    controller_proc = Process(target=run_controller_worker, args=(SHM_NAME,))

    hailo_proc.start()
    controller_proc.start()

    try:
        hailo_proc.join()
        controller_proc.join()
    except KeyboardInterrupt:
        hailo_proc.terminate()
        controller_proc.terminate()