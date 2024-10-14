#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import rospy
from std_msgs.msg import Bool

pub = rospy.Publisher('camera_obj_detection', Bool, queue_size=10)
rospy.init_node('camera_detection', anonymous=True)

# Load YOLOv8 model (using the nano model for speed)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = YOLO('yolov8n.pt')
model.to(device)

# Configure RealSense pipeline to stream color and depth frames
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

def get_depth_at_pixel(depth_frame, x, y):
    """
    Get the depth value at a specific pixel coordinate.
    """
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_value = depth_image[y, x]

    # Convert depth to meters (depth is in millimeters)
    depth_in_meters = depth_value / 1000.0

    return depth_in_meters

try:
    while True:
        # Wait for a coherent pair of frames: color and depth
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # If no frame is available, skip the iteration
        if not color_frame or not depth_frame:
            continue

        # Convert RealSense color frame to numpy array for OpenCV
        color_image = np.asanyarray(color_frame.get_data())

        # Use YOLOv8 to detect objects in the frame (optional, you can remove this part if you don't need YOLO)
        results = model.predict(color_image, conf=0.5)
        annotated_frame = results[0].plot()

        # Define the y-coordinate (middle of the screen)
        middle_y = color_image.shape[0] // 2

        # Define six x-coordinates (equally spaced horizontally across the screen)
        width = color_image.shape[1]
        x_points = [
            width // 7, 2 * width // 7, 3 * width // 7, 4 * width // 7,
            5 * width // 7, 6 * width // 7
        ]

        # Loop over the six points
        for i, x in enumerate(x_points):
            # Get the depth at the current point
            depth_at_point = get_depth_at_pixel(depth_frame, x, middle_y)

            # Draw a circle at the point
            cv2.circle(annotated_frame, (x, middle_y), 5, (0, 255, 0), -1)

            # Display the depth value next to the point
            cv2.putText(annotated_frame, f"{depth_at_point:.2f}m", (x - 50, middle_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            
            # Send object detection data to arduino through ROS publisher
            detection = 0
            if depth_at_point > 0 and depth_at_point < 1.0:
                detection = 1
                pub.publish(detection)
                print("pizdets blyat")

        # Display the frame with the six points and their distances
        cv2.imshow('Six Points Distance Measurement', annotated_frame) 

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pipeline.stop()
    cv2.destroyAllWindows()

finally:
    # Stop the pipeline and close OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()
