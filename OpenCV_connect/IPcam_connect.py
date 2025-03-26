import cv2
import numpy as np

cap = cv2.VideoCapture("C:/Users/ray62/Videos/2025-03-21 13-33-15.mp4")
#capture = cv2.VideoCapture('rtsp://username:password@192.168.1.64/1')
#capture = cv2.VideoCapture('http://admin:admin@192.168.124.125:8081')
#cap = cv2.VideoCapture(1)

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()