import cv2

# path1 = "/home/roisten/catkin_ws/src/yolo/videos/road.mp4"
path1 = "/home/roisten/catkin_ws/src/yolo/Tests/Line.webm"

cap = cv2.VideoCapture(path1)

if not cap.isOpened():
    print("Error pizdets")


while True:
    ret, frame = cap.read()
    if not ret:
        break

    resized_frame = cv2.resize(frame, (800, 800))
    cv2.imshow("Lane", resized_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
     
