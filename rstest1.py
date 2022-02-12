
import numpy as np
import pyrealsense2 as rs
import json

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)


def get_aligned_images():
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
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame


if __name__ == "__main__":
    dis_before = 0
    while 1:
        intr, depth_intrin, rgb, depth, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
        # 定义需要得到真实三维信息的像素点（x, y)，本例程以中心点为例
        x = 200
        y = 200
        # dis=0
        dis = aligned_depth_frame.get_distance(x, y)

        # dis=1  # （x, y)点的真实深度值
        # （x, y)点在相机坐标系下的真实值，为一个三维向量。其中camera_coordinate[2]仍为dis，camera_coordinate[0]和camera_coordinate[1]为相机坐标系下的xy真实距离。
        if dis:
            camera_coordinate = rs.rs2_deproject_pixel_to_point(
                depth_intrin, [x, y], dis)
            camera_coordinate = np.round(
                np.array(camera_coordinate), 3)  # 转成2位小数
            cv2.putText(rgb, str(camera_coordinate), (x+20, y+10), 0, 1,
                        [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
            print(camera_coordinate)
            dis_before = dis
        '''
        else:
            camera_coordinate = rs.rs2_deproject_pixel_to_point(
                depth_intrin, [x, y], dis_before)
            camera_coordinate = np.round(
                np.array(camera_coordinate), 2)  # 转成2位小数
            cv2.putText(rgb, str(camera_coordinate), (x, y), 0, 1,
                        [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
'''
        cv2.circle(rgb, (x, y), 4, (255, 255, 255), 5)  #
        

        cv2.namedWindow('RGB image', flags=cv2.WINDOW_NORMAL |
                        cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

        cv2.imshow('RGB image', rgb)  # 显示彩色图像

        key = cv2.waitKey(20)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            pipeline.stop()
            break
    cv2.destroyAllWindows()
