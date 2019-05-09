# test stereo cam
import cv2
import numpy as np

cam_cap = 0

cam_left_id = int(input('please input cam_left id\n'))
cam_left = cv2.VideoCapture(cam_cap + cam_left_id)

cam_right_id = int(input('please input cam_right id\n'))
cam_right = cv2.VideoCapture(cam_cap + cam_right_id)

while True:
    im1 = cam_left.read()[1]
    im2 = cam_right.read()[1]
    im = np.concatenate([im1, im2], 1)
    cv2.imshow('v', im)
    cv2.waitKey(1)
