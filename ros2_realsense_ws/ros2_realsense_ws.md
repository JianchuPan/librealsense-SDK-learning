# ros2_realsense_ws:use realsense ros Wrapper(realsense_camera) in ROS2
## 基于Realsense2 SDK提供的ros扩展功能包进行相机的使用

### 一、依赖
1. **librealsense2** : realsense相机的SDK-[librealsense](https://github.com/IntelRealSense/librealsense)
2. **realsense2_camera** : 提供的一个用于ros的扩展功能包-[realsense2_camera](https://github.com/IntelRealSense/realsense-ros)

### 安装
1. 安装**install librealsense2**
```
    >>sudo apt update
    >>sudo apt install -y librealsense-dkms librealsense2-Utils 
```

**[注]：** 以上无法安装时，可克隆[librealsense](https://github.com/IntelRealSense/librealsense)进行编译、安装，自定义路径安装后注意配置环境变量：
```
export CMAKE_PREFIX_PATH=~/具体安装路径/librealsense/lib/cmake/realsense2:$CMAKE_PREFIX_PATH
```

2.  安装**realsense_camera**
```
>>sudo apt install -y ros-${ROS_DISTRO}-realsense-camera
```

### 二、基本使用
1. realsense_py功能包的test1_image_processor
    1. 启动realsense相机，会发布话题(默认是会有rgb、深度相关的一些话题，详细见[realsense2_camera](https://github.com/IntelRealSense/realsense-ros)介绍)
    ```
    >>ros2 launch realsense2_camera rs_launch.py
    ```
    2. 启动test1_image_processor，运行自己定义的节点(订阅rgb、depth话题，并提供保存图像的服务)
    ```
    >>ros2 run realsense_py test1_image_processor
    ```
    3. 请求服务以保存图片
    ```
    >>ros2 service call /save_realsense_images std_srvs/srv/Empty 
    ```