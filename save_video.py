import numpy as np
import cv2
import os
import copy
from filterpy.kalman import KalmanFilter
from numpy.linalg import inv
import matplotlib.pyplot as plt

filename = "second/first"
width, height = 640, 480

vout_l = cv2.VideoWriter(filename+"_L.avi", cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))
vout_r = cv2.VideoWriter(filename+"_R.avi", cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

cap_l = cv2.VideoCapture(2)
cap_r = cv2.VideoCapture(4)

while(True):
    _, image_l = cap_l.read()
    _, image_r = cap_r.read()

    vout_l.write(image_l)
    vout_r.write(image_r)

    cv2.imshow("saving", cv2.hconcat([image_l, image_r]))

    if cv2.waitKey(25) == ord('q'):
        break