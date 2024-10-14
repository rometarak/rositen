import pyrealsense2 as rs
import numpy as np
import cv2

def process_frame(color_frame):
    """Processes the frame to detect white lines."""
    color_image = np.asanyarray(color_frame.get_data())
    
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(thresholded, 50, 150)

    # Detect lines using Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=30, maxLineGap=10)
    
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            # Calculate line's midpoint and angle
            line_angle, line_position = calculate_line_angle_and_position(x1, y1, x2, y2, color_image.shape)
            control_robot(line_angle, line_position)

            # Draw the line and label the vector
            cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color_image, f"({x1},{y1}) -> ({x2},{y2})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return color_image

def calculate_line_angle_and_position(x1, y1, x2, y2, image_shape):
    """Calculate the angle and position of the detected line relative to the image center."""
    image_center = image_shape[1] // 2  # Image width / 2
    line_midpoint = (x1 + x2) // 2
    
    # Calculate the angle of the line in degrees
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
    
    # Calculate how far the line's midpoint is from the center of the image
    position_offset = line_midpoint - image_center

    return angle, position_offset

def control_robot(angle, position_offset):
    """Control logic to move the robot based on the line angle and position."""
    if abs(position_offset) < 50 and abs(angle) < 10:
        # Line is centered and roughly vertical -> move forward
        print("Move forward")
        # Send command to robot to move forward
    elif position_offset < 0 or angle < 0:
        # Line is to the left -> turn left
        print("Turn left")
        # Send command to robot to turn left
    else:
        # Line is to the right -> turn right
        print("Turn right")
        # Send command to robot to turn right

# Configure and start the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            processed_image = process_frame(color_frame)
            cv2.imshow('Line Detection for Robot', processed_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pipeline.stop()
    cv2.destroyAllWindows()
    
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
