import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
# 创建 config 对象：
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)
point = 400, 70

while True:

    # Wait for a coherent pair of frames（一对连贯的帧）: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    distance = depth_frame.get_distance(point[0],point[1])
    print(distance)

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # print(type(depth_colormap))
    # <class 'numpy.ndarray'>

    # Stack both images horizontally（水平堆叠两个图像）
    images = np.hstack((color_image, color_image))
    cv2.circle(images, point, 1, (255, 255, 255), 5)  #
    cv2.putText(images, str(distance), (point[0] + 2, point[1] + 2), 0, 0.5, [225, 255, 255], thickness=1,
                lineType=cv2.LINE_AA)

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    cv2.imshow('RealSense', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # print(cv2.waitKey(1))
