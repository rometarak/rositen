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

current_velocity = 0.0

def get_depth_at_pixel(depth_frame, x, y):
    """
    Get the depth value at a specific pixel coordinate.
    """
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_value = depth_image[y, x]

    # Convert depth to meters (depth is in millimeters)
    depth_in_meters = depth_value / 1000.0

    return depth_in_meters

def calculate_braking_force(distance, velocity):
    """
    Calculate braking deceleration using the formula a = v^2 / (2 * d).
    """
    if distance > 0:
        deceleration = (velocity ** 2) / (2 * distance)
        return deceleration
    return 0.0

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

        detection = False

        # Loop over each detected object in the frame
        for obj in results[0].boxes:
            # Get the bounding box coordinates
            x1, y1, x2, y2 = map(int, obj.xyxy[0])

            # Calculate the center of the bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Get the depth at the center of the bounding box
            depth_at_object = get_depth_at_pixel(depth_frame, center_x, center_y)

            # Display depth information on the annotated frame
            cv2.putText(annotated_frame, f"{depth_at_object:.2f}m", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Determine if an object is within a dangerous range (e.g., within 1 meter)
            if 0 < depth_at_object < 1.5 :
                detection = True

            # Calculate the required braking force or speed reduction
            braking_force = calculate_braking_force(depth_at_object, current_velocity)
            print(f"Object at {depth_at_object:.2f}m: Required braking force: {braking_force:.2f} m/s²")

            # Publish the braking force/speed to ROS
            #brake_pub.publish(braking_force)

        # Publish object detection information to ROS
        pub.publish(detection)

        # Display the frame with the bounding boxes and distances
        cv2.imshow('Object Detection and Distance', annotated_frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
except KeyboardInterrupt:
    print("Exiting program...")
    
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
