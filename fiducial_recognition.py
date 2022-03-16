import numpy as np
import pupil_apriltags as at
import cv2
import os 
import math
from time import *
from utlis import *
from params import *

estimate_tag_pose = True

cam_params = np.array([611.8989, 611.8647, 320.8165, 238.4556]) # 单位像素
tag_size = 0.075 # 单位米，只黑色边框的size
#dcoeffs = np.zeros(5)
current_dir = os.getcwd()

# Image Detection
img_dir = os.path.join(current_dir,'0000.jpg') 


img = cv2.imread(img_dir,cv2.IMREAD_COLOR) # Read image
# height, width, channels = img.shape
# print(height, width, channels)
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Underdistor (Apriltag 默认输入图像为去畸变图像)
gray_undst = cv2.undistort(gray_img, cameraMatrix1, distCoeffs1, None, cameraMatrix1)
img_undst = cv2.undistort(img, cameraMatrix1, distCoeffs1, None, cameraMatrix1)

try:
	img.shape
except:
	print("can not read the image")

begin_time = time()
detector = at.Detector(families='tag16h5') # Detect fiducial
result = detector.detect(gray_undst, estimate_tag_pose, cam_params, tag_size)
# pdb.set_trace()
# H = result[0].homography
R = result[0].pose_R # R =R_z*R_y*R_x  表示从原系按XYZ顺序旋转得到目标系
euler = rotationMatrix2EulerAngles(R)
T = result[0].pose_t*1000 # 单位 m 和MATLAB 默认单位不一样
Distance = math.sqrt((T[0]*T[0])+(T[1]*T[1])+(T[2]*T[2]))
pose = RT2Homegenous34(R,T)
# draw_pose_box(img_undst, cam_params, None, tag_size, pose, z_sign=1)

xyz_wo = np.array([[50,0,0,1],[0,50,0,1],[0,0,50,1]]).transpose() # 世界坐标系
xyz_c = np.dot(pose, xyz_wo) # 相机坐标系
xyz_repro = np.dot(cameraMatrix1, xyz_c) 
xy_im = xyz_repro[0:2,:] # 像素坐标系
z_c = xyz_repro[2,:] # 比例（深度）
xy_im = xyz_repro[0:2,:]/z_c # 像素坐标系

# print('H={}, pt1{}, pt2{}'.format(H, pt, result[0].center))
# cv2.circle(img, result[0].corners[0].astype(int), 4, (255, 0, 0), 2) # left-top
# cv2.circle(img, result[0].corners[1].astype(int), 4, (255, 0, 0), 2) # right-top
# cv2.circle(img, result[0].corners[2].astype(int), 4, (255, 0, 0), 2) # right-bottom
# cv2.circle(img, result[0].corners[3].astype(int), 4, (255, 0, 0), 2) # left-bottom

cv2.line(img_undst, result[0].center.astype(int),xy_im[:,0].transpose().astype(int), (255, 0, 0), 1, 16) # x 轴
cv2.line(img_undst, result[0].center.astype(int),xy_im[:,1].transpose().astype(int), (0, 255, 0), 1, 16) # y 轴
cv2.line(img_undst, result[0].center.astype(int),xy_im[:,2].transpose().astype(int), (0, 0, 255), 1, 16) # z 轴

end_time = time()
run_time = end_time-begin_time

print(' Runtime:\n{}\n Rotation Matrix:\n{}\n Euler Angle:\n{}\n Translation Vector(mm):\n{}\n Distance(mm):\n{}'.format(run_time, R, euler, T, Distance))
cv2.namedWindow('fiducial',cv2.WINDOW_NORMAL) #O表示显示窗口可以随意手动调节，1
cv2.imshow('fiducial',img_undst)
cv2.waitKey()
cv2.destroyAllWindows()


