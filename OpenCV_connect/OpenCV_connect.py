import cv2
import numpy as np

#capture = cv2.VideoCapture('rtsp://username:password@192.168.1.64/1')

#capture = cv2.VideoCapture('rtsp://username:password@192.168.1.64/1')
cap = cv2.VideoCapture(1)

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()