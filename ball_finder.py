import numpy as np
from multiprocessing.shared_memory import SharedMemory
import config

def ball_finder(shared_ball_pos):

    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    Gst.init(None)

    pipeline = Gst.parse_launch(config.GST_PIPELINE_STR)
    appsink = pipeline.get_by_name("sink")

    def on_new_sample(sink):
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()
        from hailo import HailoROIPooled 
        roi = HailoROIPooled.get_hailo_main_object(buffer)
        objects = roi.get_objects()
        best_ball = None
        max_confidence = 0.0
        for obj in objects:
            confidence = obj.get_confidence()
            if confidence > max_confidence:
                max_confidence = confidence
                best_ball = obj

        if best_ball:
            bbox = best_ball.get_bbox()
            center_x = bbox.xmin() + (bbox.width() / 2.0)
            center_y = bbox.ymin() + (bbox.height() / 2.0)
            shared_ball_pos[0] = center_x
            shared_ball_pos[1] = center_y
        else:
            shared_ball_pos[0] = -1.0
            shared_ball_pos[1] = -1.0

        return Gst.FlowReturn.OK
    appsink.connect("new-sample", on_new_sample)

    pipeline.set_state(Gst.State.PLAYING)
    
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
