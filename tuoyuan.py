import cv2
import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import math


def nothing(x):
    pass


def get_aligned_images(pipeline, align):
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    # json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame


def circle_detect_init():
    cv2.namedWindow('control')
    cv2.createTrackbar('threshold', 'control', 127, 255, nothing)
    cv2.createTrackbar('maxRadius', 'control', 150, 200, nothing)
    cv2.createTrackbar('minRadius', 'control', 90, 200, nothing)
    t = 127

def blob_detect(im):
    im=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 100;
    params.maxThreshold = 200;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.75

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector_create(params)
    # detector = cv2.SimpleBlobDetector()
    # Detect blobs.
    keypoints = detector.detect(im)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)

def circle_dectect(img):
    # img_copy = img.copy()
    # img_copy = cv2.medianBlur(img_copy, 5)  # 中值滤波
    # gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)  # 灰度图
    # t = cv2.getTrackbarPos('threshold', 'control')  # 得到滤波阈值
    # minRadius = cv2.getTrackbarPos('minRadius', 'control')  # 最小的半径
    # maxRadius = cv2.getTrackbarPos('maxRadius', 'control')  # 最大的半径
    # flag, binary = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)  # 二值化
    # k = np.ones((3, 3), np.int8)  # 滤波kenerl
    # binary = cv2.erode(binary, k)  # 腐蚀
    # binary = cv2.dilate(binary, k)  # 膨胀
    #
    # cv2.imshow("gray", binary)  # 二值化show
    binary=img
    circles = cv2.HoughCircles(binary, cv2.HOUGH_GRADIENT, 1, 70, param1=100, param2=25, minRadius=10,
                               maxRadius=100)  # hough圆
    print(circles)
    x = 0
    y = 0
    if np.any(circles):
        circles = np.uint(np.around(circles))
        for i in circles[0]:
            x, y, r = i
            cv2.circle(img, (x, y), r, (0, 0, 255), 3)
            cv2.circle(img, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(img, str((x, y)), (x + 20, y + 10), 0, 0.5,
                        [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)

    return x, y


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
    ux = 0
    uy = 0
    for cnt in contours:
        if len(cnt) > 50:
            # S1 = cv2.contourArea(cnt)  # 格林公式计算的实际面积
            ell = cv2.fitEllipse(cnt)  # 拟合椭圆 ellipse = [ center(x, y) , long short (a, b), angle ]
            x = int(ell[0][0])
            y = int(ell[0][1])
            a = ell[1][0]
            b = ell[1][1]
            # S2 = math.pi * ell[1][0] * ell[1][1]  # 理论面积
            if (b / a) < 1.2 and a > 80 and b > 80 and a < 120 and b < 120:  # 面积比例
                uy = y
                ux = x
                img = cv2.ellipse(img, ell, (0, 0, 200), 2)
                cv2.circle(img, (x, y), 2, (255, 255, 255), 3)
                cv2.putText(img, str((x, y)), (x + 20, y + 10), 0, 0.5,
                            [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)
                print("长轴： " + str(a) + "    " + "短轴： " + str(b) + "   " + str(ell[0][0]) + "   " + str(ell[0][1]))
    cv2.imshow("ell", img)
    return ux, uy


def depth_detect(aligned_depth_frame, depth_intrin, ux, uy, canvas):
    dis = aligned_depth_frame.get_distance(ux, uy)
    camera_xyz = rs.rs2_deproject_pixel_to_point(
        depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
    camera_xyz = np.round(np.array(camera_xyz), 3)  # 转成3位小数
    camera_xyz = camera_xyz.tolist()
    cv2.circle(canvas, (ux, uy), 4, (255, 255, 255), 5)  # 标出中心点
    cv2.putText(canvas, str(camera_xyz), (ux + 20, uy + 10), 0, 1,
                [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)  # 标出坐标
