import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline to configure and start the RealSense camera
pipeline = rs.pipeline()
config = rs.config()

# Enable the streams from RealSense (depth and color)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for frames from the RealSense camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        # Display the images in a window
        cv2.imshow('RealSense RGB and Depth', color_image)
        
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming and close the window
    pipeline.stop()
    cv2.destroyAllWindows()
