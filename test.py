import gi
gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib

Gst.init(None)

pipeline = Gst.parse_launch("""
libcamerasrc !
videoconvert !
appsink name=sink emit-signals=true
""")

sink = pipeline.get_by_name("sink")

def cb(sink):
    print("FRAME")
    sink.emit("pull-sample")
    return Gst.FlowReturn.OK

sink.connect("new-sample", cb)

pipeline.set_state(Gst.State.PLAYING)

GLib.MainLoop().run()