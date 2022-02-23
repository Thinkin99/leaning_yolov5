import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('1.jpg', 0)

orb = cv2.ORB_create()
img_copy = img.copy()
img_copy = cv2.medianBlur(img_copy, 5)
#gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)

kp = orb.detect(img_copy, None)

kp, des = orb.compute(img_copy, kp)

img_copy = cv2.drawKeypoints(img_copy, kp, img, color=(0, 255, 0), flags=0)

cv2.imshow('p', img_copy)

cv2.waitKey()
