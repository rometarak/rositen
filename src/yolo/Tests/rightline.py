import pyrealsense2 as rs
import numpy as np
import cv2

def apply_birds_eye_view(image):
    """Applies a bird's-eye view transformation to the given image."""
    height, width = image.shape[:2]
    
    # Define source points (corners of the region in the original image)
    src_points = np.float32([
        [width * 0.3, height * 0.9],  # Bottom left
        [width * 0.7, height * 0.9],  # Bottom right
        [width * 0.2, height * 0.6],  # Top left
        [width * 0.8, height * 0.6]   # Top right
    ])
    
    # Define destination points (rectangle for bird's-eye view)
    dst_points = np.float32([
        [0, height],         # Bottom left
        [width, height],     # Bottom right
        [0, 0],              # Top left
        [width, 0]           # Top right
    ])
    
    # Calculate the perspective transformation matrix
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    
    # Apply the perspective warp to get a bird's-eye view
    warped = cv2.warpPerspective(image, matrix, (width, height))
    
    return warped

def filter_lines(lines, width):
    """Separate lines into left and right based on their slope and position."""
    left_lines = []
    right_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero

        if slope < -0.3 and x1 < width / 2 and x2 < width / 2:
            left_lines.append(line)
        elif slope > 0.3 and x1 > width / 2 and x2 > width / 2:
            right_lines.append(line)

    return left_lines, right_lines

def average_lane(lines, height):
    """Averages the position of lines to form a single line for a lane."""
    if not lines:
        return None

    x1s, y1s, x2s, y2s = [], [], [], []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        x1s.append(x1)
        y1s.append(y1)
        x2s.append(x2)
        y2s.append(y2)

    # Average values
    x1 = int(np.mean(x1s))
    y1 = int(np.mean(y1s))
    x2 = int(np.mean(x2s))
    y2 = height  # Extend the line to the bottom of the image

    return [(x1, y1, x2, y2)]

def draw_lane_lines(image, lane, color):
    """Draws a line representing a lane on the image."""
    if lane is None:
        return
    for line in lane:
        x1, y1, x2, y2 = line
        cv2.line(image, (x1, y1), (x2, y2), color, 5)

def process_frame(color_frame):
    """Processes the frame to detect and draw lanes and guide the vehicle."""
    color_image = np.asanyarray(color_frame.get_data())
    
    # Apply bird's-eye view transformation
    birdseye_view = apply_birds_eye_view(color_image)
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(birdseye_view, cv2.COLOR_BGR2GRAY)
    
    # Threshold and edge detection
    _, thresholded = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(thresholded, 150, 255)

    # Detect lines using Hough Transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=150)

    height, width = birdseye_view.shape[:2]  # Get image dimensions

    if lines is not None:
        left_lines, right_lines = filter_lines(lines, width)
        left_lane = average_lane(left_lines, height)
        right_lane = average_lane(right_lines, height)

        # Draw the left and right lanes
        if left_lane:
            draw_lane_lines(birdseye_view, left_lane, color=(0, 0, 255))  # Red for the left lane
        if right_lane:
            draw_lane_lines(birdseye_view, right_lane, color=(0, 255, 0))  # Green for the right lane

        # Calculate and draw the midpoint, biased to the right
        if left_lane and right_lane:
            # Adjust weights to favor the right lane (e.g., 70% towards the right, 30% towards the left)
            weight_left = 0.3
            weight_right = 0.7

            mid_x1 = int((weight_left * left_lane[0][0] + weight_right * right_lane[0][0]))
            mid_x2 = int((weight_left * left_lane[0][2] + weight_right * right_lane[0][2]))

            cv2.line(birdseye_view, (mid_x1, 0), (mid_x2, height), (255, 0, 0), 2)  # Blue for the biased center line

    # Show the result in a window
    cv2.imshow('Lane Detection with Right Bias', birdseye_view)
    return birdseye_view

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
            
            # Display the processed frame
            cv2.imshow('Lane Detection with Right Bias', processed_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pipeline.stop()
    cv2.destroyAllWindows()
