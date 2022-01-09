import sys
import cv2


camSet='nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1280, height=720, framerate=21/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw,width=960, height=616, format=BGRx ! videoconvert ! appsink'

video = cv2.VideoCapture(camSet)

while(True):
    ret, frame = video.read()

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()

cv2.destroyAllWindows()
