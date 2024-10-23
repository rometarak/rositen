import gi 
import sys
from time import sleep

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

Gst.init(sys.argv)

pipeline = Gst.parse_launch('v4l2src device=/dev/video4 ! video/x-raw,width=640,height=480 ! queue ! jpegenc ! rtpjpegpay ! udpsink host=10.0.3.60 port=5004')

def main():
  bus = pipeline.get_bus()      
  pipeline.set_state(Gst.State.PLAYING)
  print('Dashcam Live')
  
  
  bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
  pipeline.set_state(Gst.State.NULL)
try:
  main()
except KeyboardInterrupt:
  print("Interrupt received, stopping...")
finally:
    # Cleanup: set pipeline state to NULL
    pipeline.set_state(Gst.State.NULL)
    print("Pipeline stopped")