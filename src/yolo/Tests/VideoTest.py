
import cv2
import numpy as np


cap = cv2.VideoCapture('/dev/video4') 
success = True

def nothing(x):
    pass

cv2.namedWindow("Trackbars")

cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing) #0 ja valjas 81
cv2.createTrackbar("U - H", "Trackbars", 159, 255, nothing) #196 ja valjas 219
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing) #255 ja valjas 216
cv2.createTrackbar("U - V", "Trackbars", 216, 255, nothing) #216 ja valjas 120



while True:
    success, image = cap.read()
    if success:
        frame = cv2.resize(image, (640,480))
    
        
        tl = (80, 300)
        tr = (560, 300)
        bl = (20, 472)
        br = (620, 472)

        cv2.circle(frame, tl, 5, (0,0,255),-1)
        cv2.circle(frame, bl, 5, (0,0,255),-1)
        cv2.circle(frame, tr, 5, (0,0,255),-1)
        cv2.circle(frame, br, 5, (0,0,255),-1)

        pts1 = np.float32([tl,bl,tr,br])
        pts2 = np.float32([[0,0], [0, 480], [640, 0], [640, 480]])

        matrix = cv2.getPerspectiveTransform(pts1,pts2)
        transformed_frame = cv2.warpPerspective(frame,matrix, (640,480))

        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        lower  = np.array([l_h, l_s, l_v])
        higher = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv_transformed_frame, lower, higher)

        res = cv2.bitwise_and(transformed_frame, transformed_frame, mask=mask)

        cv2.imshow('Line', hsv_transformed_frame)
        #cv2.imshow('Birdseye', transformed_frame)
        cv2.imshow("skibidi", res)
 

    # Exit on 'q' key press
    if cv2.waitKey(29) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
