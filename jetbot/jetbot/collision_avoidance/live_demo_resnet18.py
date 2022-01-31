import torch
import torchvision

import os

print(">>>>>>>>>>>" + os.getcwd())

model = torchvision.models.resnet18(pretrained=False)
model.fc = torch.nn.Linear(512, 2)

model.load_state_dict(torch.load('jetbot/jetbot/collision_avoidance/best_model_resnet18.pth'))

device = torch.device('cuda' if torch.cuda.is_available() else "cpu")
model = model.to(device)
model = model.eval().half()

import torchvision.transforms as transforms
import torch.nn.functional as F
import cv2
import PIL.Image
import numpy as np

if torch.cuda.is_available():
    mean = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
    std = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()
else:
    mean = torch.Tensor([0.485, 0.456, 0.406]).half()
    std = torch.Tensor([0.229, 0.224, 0.225]).half()

normalize = torchvision.transforms.Normalize(mean, std)

def preprocess(image):
    image = PIL.Image.fromarray(image)
    image = transforms.functional.to_tensor(image).to(device).half()
    image.sub_(mean[:, None, None]).div_(std[:, None, None])
    return image[None, ...]


import torch.nn.functional as F
import time

def update(change):
    '''global blocked_slider, robot
    x = change['new'] 
    x = preprocess(x)
    y = model(x)
    
    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    
    prob_blocked = float(y.flatten()[0])
    
    blocked_slider.value = prob_blocked
    
    if prob_blocked < 0.5:
        robot.forward(speed_slider.value)
    else:
        robot.left(speed_slider.value)
    
    '''
    x = change['new'] 
    x = preprocess(x)
    y = model(x)
    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)

    print(y)
    time.sleep(0.001)
        

# prepare the camera set
#camSet='nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1280, height=720, framerate=21/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw,width=960, height=616, format=BGRx ! videoconvert ! appsink'
camSet=0

# start the camera capture
video = cv2.VideoCapture(camSet)
# the window name use when displaying then updating the window title
window_name = "frame"

while(True):
    # read video and show image
    ret, frame = video.read()
    cv2.imshow(window_name, frame)

    update({'new': frame})  # we call the function once to initialize

    # wait key pressed
    key = cv2.waitKey(1)

    # q = quit
    if key == ord('q'):
        break

