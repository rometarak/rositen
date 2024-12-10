import gi
import sys
from time import sleep

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# Initialize GStreamer
Gst.init(sys.argv)

# Create the pipeline
pipeline = Gst.parse_launch('v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast ! rtph264pay ! udpsink host=10.0.3.98 port=5004 sync=false')

def on_message(bus, message, loop):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End-of-stream")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err}, {debug}")
        loop.quit()

def main():
    # Create the GObject MainLoop
    loop = GObject.MainLoop()

    # Get the pipeline's bus to listen for messages
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_message, loop)

    # Set the pipeline to the playing state
    pipeline.set_state(Gst.State.PLAYING)
    print("Dashcam Live")

    try:
        # Run the loop
        loop.run()
    except KeyboardInterrupt:
        print("Interrupt received, stopping...")
    finally:
        # Cleanup: set pipeline state to NULL
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped")

if __name__ == "__main__":
    main()
