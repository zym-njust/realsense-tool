import pyrealsense2 as rs
import numpy as np
import cv2
import math
import os


def get_serail_number():
    '''Read serial number and camera name  '''
    context = rs.context()
    name = context.devices[0].get_info(rs.camera_info.name)
    serial = context.devices[0].get_info(rs.camera_info.serial_number)
    return name,serial


# aligned_frames = align.process(frames_1)
# depth_frame = aligned_frames.get_depth_frame()
# color_frame = aligned_frames.get_color_frame()

def draw_pose_box(img, camera_params, dcoeffs, tag_size, pose, z_sign=1):

    opoints = np.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size  # 世界坐标系

    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2) # edge index

    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(img, ipoints[i], ipoints[j], (0, 255, 0), 1, 8)

def RT2Homegenous44(R, T):
    '''
    Given a rotation matrix and a translation vector, return a homogeneous transformation matrix.
    
    :param R: 3x3 rotation matrix
    :param T: translation vector
    :return: The homogeneous transformation matrix.
    '''
    pose = np.zeros((4,4))
    pose[0][0] = R[0][0]
    pose[0][1] = R[0][1]
    pose[0][2] = R[0][2]
    pose[0][3] = T[0]
    pose[1][0] = R[1][0]
    pose[1][1] = R[1][1]
    pose[1][2] = R[1][2]
    pose[1][3] = T[1]
    pose[2][0] = R[2][0]
    pose[2][1] = R[2][1]
    pose[2][2] = R[2][2]
    pose[2][3] = T[2]
    pose[3,:] = [0, 0, 0, 1]
    return pose

def RT2Homegenous34(R, T):
    pose = np.zeros((3,4))
    pose[0][0] = R[0][0]
    pose[0][1] = R[0][1]
    pose[0][2] = R[0][2]
    pose[0][3] = T[0]
    pose[1][0] = R[1][0]
    pose[1][1] = R[1][1]
    pose[1][2] = R[1][2]
    pose[1][3] = T[1]
    pose[2][0] = R[2][0]
    pose[2][1] = R[2][1]
    pose[2][2] = R[2][2]
    pose[2][3] = T[2]
    return pose

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).

def rotationMatrix2EulerAngles(R):
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def mkdir(path_list):
    '''
    Create a folder if it doesn't exist.
    
    :param path_list: the path list of the file you want to create
    :return: None
    '''
    for path in path_list:
        folder = os.path.exists(path)
    
        if not folder:                   #判断是否存在文件夹如果不存在则创建为文件夹
            os.makedirs(path)            #makedirs 创建文件时如果路径不存在会创建这个路径
            print('---  new folder...  ---')
            print('---  OK  ---')
    
        else:
            print ('---  The folder already exists!  ---')
            
if __name__ == '__main__':
    name, serial = get_serail_number()
    print(name,serial)