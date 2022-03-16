# -*- coding: utf-8 -*-
"""
Created on Mon Aug 26 19:40:20 2019

@author: Admin
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from utlis import *

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming
pipe_profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

dir_root=['image/mono/']
dir_list='.jpg'
print('start')
count=300

# For binocular camera
mkdir(dir_root)


while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    img_color = np.asanyarray(color_frame.get_data())
    img_depth = np.asanyarray(depth_frame.get_data())
    
    cv2.imshow('depth_frame',img_color)
    cv2.imshow('d_frame',img_depth)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('a'):
        count0=count%10
        count1=int(count/10)%10
        count2=int(count/100)%10
        count3=int(count/1000)%10
        cv2.imwrite(dir_root+str(count3)+str(count2)+str(count1)+str(count0)+dir_list,img_color)
        count+=1
    elif key & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

pipeline.stop()
