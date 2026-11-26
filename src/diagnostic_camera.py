"""Run the existing Hailo pipeline with an on-demand JPEG preview branch."""

import signal
import sys

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst

from diagnostic_stream import CAMERA_SOCKET, DiagnosticStream

PREVIEW = """
    ! tee name=frames
    frames. ! fakesink sync=false
    frames. ! valve name=preview_gate drop=true
        ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream
        ! videorate drop-only=true ! video/x-raw,framerate=12/1
        ! hailooverlay ! videoscale ! videoconvert
        ! video/x-raw,format=I420,width=640,height=480
        ! jpegenc quality=65
        ! appsink name=preview emit-signals=true max-buffers=1 drop=true sync=false async=false
"""


def main():
    Gst.init(None)
    pipeline = Gst.parse_launch(" ".join(sys.argv[1:]) + PREVIEW)
    loop = GLib.MainLoop()
    gate = pipeline.get_by_name("preview_gate")
    sink = pipeline.get_by_name("preview")

    def set_preview(active):
        GLib.idle_add(gate.set_property, "drop", not active)

    stream = DiagnosticStream(CAMERA_SOCKET, bytes, set_preview)

    def on_sample(appsink):
        sample = appsink.emit("pull-sample")
        if sample is not None and stream.active:
            buffer = sample.get_buffer()
            stream.publish(buffer.extract_dup(0, buffer.get_size()))
        return Gst.FlowReturn.OK

    sink.connect("new-sample", on_sample)
    failed = False

    def on_error(_bus, message):
        nonlocal failed
        error, debug = message.parse_error()
        print(f"FB4 camera: {error}\n{debug or ''}", file=sys.stderr, flush=True)
        failed = True
        loop.quit()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message::error", on_error)
    bus.connect("message::eos", lambda *_: loop.quit())
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, loop.quit)
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, loop.quit)

    try:
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            return 1
        loop.run()
        return int(failed)
    finally:
        pipeline.set_state(Gst.State.NULL)
        bus.remove_signal_watch()


if __name__ == "__main__":
    sys.exit(main())
