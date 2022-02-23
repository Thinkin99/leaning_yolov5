import cv2
import numpy as np
from matplotlib import pyplot as plt
import math


def nothing(x):
    pass


def circle_detect_init():
    cv2.namedWindow('control')
    cv2.createTrackbar('threshold', 'control', 127, 255, nothing)
    cv2.createTrackbar('maxRadius', 'control', 150, 200, nothing)
    cv2.createTrackbar('minRadius', 'control', 90, 200, nothing)
    t = 127


def circle_dectect(img):
    img_copy = img.copy()
    img_copy = cv2.medianBlur(img_copy, 5)  # 中值滤波
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)  # 灰度图
    t = cv2.getTrackbarPos('threshold', 'control')  # 得到滤波阈值
    minRadius = cv2.getTrackbarPos('minRadius', 'control')  # 最小的半径
    maxRadius = cv2.getTrackbarPos('maxRadius', 'control')  # 最大的半径
    flag, binary = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)  # 二值化
    k = np.ones((3, 3), np.int8)  # 滤波kenerl
    binary = cv2.erode(binary, k)  # 腐蚀
    binary = cv2.dilate(binary, k)  # 膨胀

    cv2.imshow("gray", binary)#二值化show
    circles = cv2.HoughCircles(binary, cv2.HOUGH_GRADIENT, 1, 70, param1=100, param2=25, minRadius=minRadius,
                               maxRadius=maxRadius)#hough圆
    print(circles)
    if np.any(circles):
        circles = np.uint(np.around(circles))
        for i in circles[0]:
            x, y, r = i;
            cv2.circle(img, (x, y), r, (0, 255, 255), 3)
            cv2.circle(img, (x, y), 2, (0, 255, 255), 3)
            cv2.putText(img, str((x, y)), (x + 20, y + 10), 0, 0.5,
                        [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
    cv2.imshow("img", img)


def ell_detect(img):
    # img = cv2.imread("1.jpg", 3)
    img_copy = img.copy()
    img = cv2.blur(img, (1, 1))
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    # flag, img_copy = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    imgray = cv2.Canny(img_copy, 600, 100, 3)  # Canny边缘检测，参数可更改

    # cv2.imshow("imgray",imgray)
    ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow("thresh", thresh)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # contours为轮廓集，可以计算轮廓的长度、面积等
    for cnt in contours:
        if len(cnt) > 50:
            # S1 = cv2.contourArea(cnt)  # 格林公式计算的实际面积
            ell = cv2.fitEllipse(cnt)  # 拟合椭圆 ellipse = [ center(x, y) , long short (a, b), angle ]
            x = int(ell[0][0])
            y = int(ell[0][1])
            a = ell[1][0]
            b = ell[1][1]
            # S2 = math.pi * ell[1][0] * ell[1][1]  # 理论面积
            if (b / a) < 1.2:  # 面积比例
                img = cv2.ellipse(img, ell, (0, 0, 200), 2)
                cv2.circle(img, (x, y), 2, (255, 255, 255), 3)
                cv2.putText(img, str((x, y)), (x + 20, y + 10), 0, 0.5,
                            [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                print("长轴： " + str(a) + "    " + "短轴： " + str(b) + "   " + str(ell[0][0]) + "   " + str(ell[0][1]))
    cv2.imshow("0", img)
    return img
