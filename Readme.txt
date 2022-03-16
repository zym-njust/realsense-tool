文件说明
------------------------------------------------
params.py 标定结果,是相对MATLAB已经转置过的
------------------------------------------------
mono_cap.py 抓取单张图片
------------------------------------------------
stereo_cap.py 抓取双目相机图片，结果存放在..\image\left和..\image\right下
------------------------------------------------
fiducial_recognition.py 测二维标识物位姿相对左目相机的位姿
    1）第一次使用需要安装apriltag的python库，
    指令  pip install pulpil_apriltags,如果安装失败大部分是因为网络问题，用镜像源下载
    2）第一次运行会报错,是apriltag库的问题
        File "D:\Conda\lib\site-packages\pupil_apriltags\bindings.py", line 441, in detect
            if camera_params == None:
        ValueError: The truth value of an array with more than one element is ambiguous. Use a.any() or a.all()
        定位到这个地方把camera_params改成camera_params.all()
-----------------------------------------------
binocular_ranging.py opencv双目测距，（目前只写到双目矫正）
-----------------------------------------------
utlis.py 工具函数
-----------------------------------------------
video_cap1.py 按s每隔一秒自动采集照片,按q停止采集，再按q关闭窗口