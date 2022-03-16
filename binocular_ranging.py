import cv2
import numpy as np
import matplotlib.pyplot as plt
from params import *
from PIL import Image
import pdb
from time import *


# Inference on single picture/

lFrame = cv2.imread('fiducial2/left/0000.jpg')
rFrame = cv2.imread('fiducial2/right/0000.jpg')

h, w = lFrame.shape[:2] # both frames should be of same shape
frames = [lFrame, rFrame]

# Inference on frames(Realsense)/
# see another script...

# Params from camera calibration
camMats = [cameraMatrix1, cameraMatrix2]
distCoeffs = [distCoeffs1, distCoeffs2]

camSources = [0,1]

# The rectification process/

# Undistortion
# rectFrames = [0,0]
# for src in camSources:
#     rectFrames[src] = cv2.undistort(frames[src], camMats[src], distCoeffs[src])

# See the results
# view = np.hstack([frames[0], frames[1]])    
# rectView = np.hstack([rectFrames[0], rectFrames[1]])
# cv2.imshow('view', view)
# cv2.imshow('rectView', rectView)

# Rectify
R1 = np.zeros(shape=(3,3))
R2 = np.zeros(shape=(3,3))
P1 = np.zeros(shape=(3,3))
P2 = np.zeros(shape=(3,3))
map = [[0,0],[0,0]]
rectifyscale  = 0    # 设置为0的话，对图片进行剪裁，设置为1则保留所有原图像像素  
begin_time = time()                  
R1, R2, P1, P2, Q, roiL, roiR = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (w,h), R, T, rectifyscale, (0,0))
# 利用initUndistortRectifyMap函数计算畸变矫正和立体校正的映射变换，实现极线对齐。
Left_Stereo_Map= cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (w,h), cv2.CV_16SC2)   
Right_Stereo_Map= cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (w,h), cv2.CV_16SC2)

Left_rectified= cv2.remap(lFrame,Left_Stereo_Map[0],Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  # 使用remap函数完成映射
Right_rectified= cv2.remap(rFrame,Right_Stereo_Map[0],Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
end_time = time()
im_L=Image.fromarray(Left_rectified)
im_R=Image.fromarray(Right_rectified) # numpy 转 image 类
print(end_time-begin_time)
  
width = w*2
height = h

img_compare = Image.new('RGBA',(width, height))
img_compare.paste(im_L,box=(0,0))
img_compare.paste(im_R,box=(w,0))

for i in range(20):
    len = w/20
    plt.axhline(y=i*len, color='r', linestyle='-')
plt.imshow(img_compare)
plt.show()
cv2.waitKey(0)

# Object Detection

# Circular Hough Transformer

# Depth



