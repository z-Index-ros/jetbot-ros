import cv2
import os
from uuid import uuid1

# set the blocked and free images directories
blocked_dir = 'dataset/blocked'
free_dir = 'dataset/free'

# we have this "try/except" statement because these next functions can throw an error if the directories exist already
try:
    os.makedirs(free_dir)
    os.makedirs(blocked_dir)
except FileExistsError:
    print('Directories not created because they already exist')

# prepare the counters info
free_count = len(os.listdir(free_dir))
blocked_count = len(os.listdir(blocked_dir))

# save functions
def save_snapshot(directory):
    image_path = os.path.join(directory, str(uuid1()) + '.jpg')
    cv2.imwrite(image_path, frame)

def save_free():
    global free_dir, free_count
    save_snapshot(free_dir)
    free_count = len(os.listdir(free_dir))
    
def save_blocked():
    global blocked_dir, blocked_count
    save_snapshot(blocked_dir)
    blocked_count = len(os.listdir(blocked_dir))

# prepare the camera set
camSet='nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1280, height=720, framerate=21/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw,width=960, height=616, format=BGRx ! videoconvert ! appsink'

# start the camera capture
video = cv2.VideoCapture(camSet)

# the window name use when displaying then updating the window title
window_name = "frame"

# show the image in a window, save images when pressing 'b' or 'f', quit when pressing 'q'
while(True):
    # read video and show image
    ret, frame = video.read()
    cv2.imshow(window_name, frame)

    # wait key pressed
    key = cv2.waitKey(1)

    # b = save blocked image
    if  key == ord('b'):
        save_blocked()

    # f = save free image
    if key == ord('f'):
        save_free()

    # q = quit
    if key == ord('q'):
        break
    
    # set the window title, showing the images count
    cv2.setWindowTitle(	window_name, 'Data Collection count: ' + str(free_count) + ' free / ' + str(blocked_count) + ' blocked') 

# release vide and close the window
video.release()
cv2.destroyAllWindows()
