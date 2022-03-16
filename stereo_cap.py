
import pyrealsense2 as rs
import numpy as np
import cv2
from utlis import *

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('827112070557')
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('943222074151')
config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming
pipe_profile_1 = pipeline_1.start(config_1)
pipe_profile_2 = pipeline_2.start(config_2)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

dir_root='image/'
dir_list='.jpg'
count=0

# For binocular camera
dir_root= ['image/left/','image/right/']

mkdir(dir_root)

# For binocular camera
dir_root_1='image/left/'
dir_root_2='image/right/'


print('start')
while True:
    # Camera 1
    frames_1 = pipeline_1.wait_for_frames()
    depth_frame_1 = frames_1.get_depth_frame()
    color_frame_1 = frames_1.get_color_frame()

    # Convert images to numpy arrays
    color_image_1 = np.asanyarray(color_frame_1.get_data())
    depth_image_1 = np.asanyarray(depth_frame_1.get_data())

    # Camera 2
    frames_2 = pipeline_2.wait_for_frames()
    depth_frame_2 = frames_2.get_depth_frame()
    color_frame_2 = frames_2.get_color_frame()

    # Convert images to numpy arrays
    color_image_2 = np.asanyarray(color_frame_2.get_data())
    depth_image_2 = np.asanyarray(depth_frame_2.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.5), cv2.COLORMAP_JET)
    depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.5), cv2.COLORMAP_JET)

     # Stack all images horizontally
    color = np.concatenate((color_image_1,color_image_2),1)
    depth = np.concatenate((depth_colormap_1,depth_colormap_2),1)
    images = np.hstack((color_image_1, depth_colormap_1,color_image_2, depth_colormap_2))   

    cv2.imshow('RealSense1', color)
    # cv2.imshow('RealSense2', depth)   
    # cv2.imshow('RealSense', images)   


    ### binocamera calibration 
    key = cv2.waitKey(1)
    if key & 0xFF == ord('a'):
        count0=count%10
        count1=int(count/10)%10
        count2=int(count/100)%10
        count3=int(count/1000)%10
        cv2.imwrite(dir_root_1+str(count3)+str(count2)+str(count1)+str(count0)+dir_list,color_image_1)
        cv2.imwrite(dir_root_2+str(count3)+str(count2)+str(count1)+str(count0)+dir_list,color_image_2)
        count+=1
        print('save')

    
    elif key & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

pipeline_1.stop()
pipeline_2.stop()
