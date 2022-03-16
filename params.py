import numpy as np

### Result of Camera Calibration
# MATLAB 中的旋转矩阵需要转置
cameraMatrix1 = np.array([[611.8989,0,320.8165],[0,611.8647,238.4556],[0,0,1]])
distCoeffs1 = np.array([[0.0984,-0.1257,-0.0028,0.0015]])
cameraMatrix2 = np.array([[605.2254,0,317.6095],[0,605.4378,250.2700],[0,0,1]])
distCoeffs2 = np.array([[0.1263,-0.2089,-0.0018,-0.0014]])
imageSize = [480,640]
R = np.array([[1,-0.0034,0.0031],[0.0033,1,0.0065],[-0.0031,-0.0065,1]])
T = np.array([-150.0272,-0.1653,-0.4840]) # 单位 毫米