import cv2
from ultralytics import YOLO


model = YOLO('yolov8n.pt')

# Load the video
video_path = "traffic_cones/Torbik.webm"
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Process each frame of the video
while cap.isOpened():
    ret, frame = cap.read()

    # Break the loop if no more frames are available
    if not ret:
        break

    # Use the model to make predictions
    results = model(frame, conf=0.25)  # Set confidence threshold

    # Iterate through the predictions and draw bounding boxes
    for result in results:
        for box in result.boxes:  # Get bounding box predictions
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get box coordinates
            conf = box.conf[0]  # Confidence score
            class_id = int(box.cls[0])  # Class ID
            
            # Define the bounding box color and thickness
            color = (255, 0, 0)  # Change as needed
            thickness = 2
            
            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            
            # Add label text above the bounding box
            label = f'Class {class_id}: {conf:.2f}'  # Modify label as needed
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    # Display the frame with the predictions
    cv2.imshow("Traffic Cone Detection", frame)

    # Press 'q' to stop the video early
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close display window
cap.release()
cv2.destroyAllWindows()
