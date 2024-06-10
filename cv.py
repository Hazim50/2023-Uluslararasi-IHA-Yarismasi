import time
import cv2

# cap=cv2.VideoCapture('rtsp://192.168.43.1:8554/fpv_stream')
cap=cv2.VideoCapture('rtsp://192.168.42.129:8554/fpv_stream')

while True:
    red,frame=cap.read()
    if red:
        cv2.imshow("Frame",frame)
        k=cv2.waitKey(1)
        if k==ord("q"):
            break
    # time.sleep(0.01)
cap.release()
cv2.destroyAllWindows()    
