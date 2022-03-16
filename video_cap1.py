
import pyrealsense2 as rs
import numpy as np
import pdb
import cv2
from utlis import *
from time import *

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config() # 创建config对象
config_1.enable_device('826212070290')
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)


# Start streaming
pipeline_1.start(config_1)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

dir_root='image/'
dir_list='.jpg'
count=0

# For binocular camera
dir_root= ['video/left/','video/right/']
mkdir(dir_root)

# For binocular camera
dir_root_1='video/left/'
dir_root_2='video/right/'

print('start')
while True:
    # Camera 1
    frames_1 = pipeline_1.wait_for_frames()
    # depth_frame_1 = frames_1.get_depth_frame()
    color_frame_1 = frames_1.get_color_frame()

    if not color_frame_1:
        continue
    # Convert images to numpy arrays
    color_image_1 = np.asanyarray(color_frame_1.get_data())
    # depth_image_1 = np.asanyarray(depth_frame_1.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)

    cv2.imshow('RealSense', color_image_1)


    ### binocamera calibration 
    key = cv2.waitKey(1)
    if key & 0xFF == ord('s'): # 自动采集
        time0 = time()
        while True:
            # 获取视频流
            frames_1 = pipeline_1.wait_for_frames()
            color_frame_1 = frames_1.get_color_frame()
            if not color_frame_1:
                continue
            color_image_1 = np.asanyarray(color_frame_1.get_data())
            cv2.imshow('RealSense', color_image_1)
            
            # 统计已存在的照片以免覆盖
            count = 0
            for filename in os.listdir('./video/left/'):
                if filename.endswith('.jpg'):
                    count += 1
            
            key = cv2.waitKey(1)
            # 等待一个时间间隔
            time_delay = 1 # (s)
            if (time() - time0) > time_delay:
                # 一个时间间隔后提取一次当前帧
                print(time() - time0)
                cv2.imwrite('./video/left/{}.jpg'.format(count + 1), color_image_1)
                time0 =time()
                
            if key & 0xFF == ord('q'):
                break

    elif key & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

pipeline_1.stop()


