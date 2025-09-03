import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
import threading
import time

class RealSenseHandler:
    def __init__(self):
        # 配置保存目录
        self.save_dir = os.path.join(os.path.expanduser('~'), 'realsense_images/realsense_standalone_images')
        os.makedirs(self.save_dir, exist_ok=True)
        print(f"图像将保存到: {self.save_dir}")
        
        # 配置相机管道
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 启用彩色和深度流（根据相机支持的分辨率调整，enable_stream(流类型, 宽度, 高度, 格式, 帧率)）
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # 最高支持1920x1080，帧率最高90FPS-->需根据分辨率来决定
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # 最高支持1280x720
        
        # 存储最新帧
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.running = False
        
        # 创建线程锁（避免多线程访问冲突）
        self.lock = threading.Lock()

    def start(self):
        """启动相机和数据采集线程"""
        self.running = True
        # 启动管道
        self.profile = self.pipeline.start(self.config)
        
        # 启动数据采集线程
        self.thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.thread.start()
        print("相机启动成功，按 's' 保存图像，按 'q' 退出。\n保存图像的s键需要直接在图像显示窗口(而非终端)中按下，\n不需要按回车键。")

    def _capture_frames(self):
        """后台线程：持续获取图像帧"""
        while self.running:
            # 等待新帧
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # 加锁更新最新帧
            with self.lock:
                self.latest_color_frame = np.asanyarray(color_frame.get_data())
                self.latest_depth_frame = np.asanyarray(depth_frame.get_data())
            
            # 短暂休眠，降低CPU占用
            time.sleep(0.01)

    def save_images(self):
        """保存当前最新的图像"""
        with self.lock:
            if self.latest_color_frame is None or self.latest_depth_frame is None:
                print("警告：没有可用图像可保存")
                return
            
            # 生成时间戳文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 保存彩色图像
            color_path = os.path.join(self.save_dir, f"color_{timestamp}.png")
            cv2.imwrite(color_path, self.latest_color_frame)
            
            # 处理深度图（归一化便于可视化）
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.latest_depth_frame, alpha=0.03),
                cv2.COLORMAP_JET
            )
            depth_path = os.path.join(self.save_dir, f"depth_{timestamp}.png")
            cv2.imwrite(depth_path, depth_colormap)
            
            # 保存原始深度数据（numpy格式）
            depth_raw_path = os.path.join(self.save_dir, f"depth_raw_{timestamp}.npy")
            np.save(depth_raw_path, self.latest_depth_frame)
            
            print(f"图像已保存: {color_path}, {depth_path}, {depth_raw_path}")

    def run(self):
        """主循环: 显示图像并处理键盘指令(替代ROS服务)"""
        while self.running:
            with self.lock:
                # 复制当前帧（避免显示时被采集线程修改）
                color_img = self.latest_color_frame.copy() if self.latest_color_frame is not None else None
                depth_img = self.latest_depth_frame.copy() if self.latest_depth_frame is not None else None
            
            # 显示图像
            if color_img is not None:
                cv2.imshow('Color Image', color_img)
            
            if depth_img is not None:
                # 实时显示深度图的彩色映射
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_img, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Depth Image', depth_colormap)
            
            # 处理键盘输入（'s'保存，'q'退出）
            key = cv2.waitKey(1) # 程序通过 OpenCV 的cv2.waitKey()函数监听图像窗口的键盘输入，而非终端输入
            if key == ord('s'):
                self.save_images()
            elif key == ord('q'):
                self.running = False

        # 清理资源
        self.pipeline.stop()
        cv2.destroyAllWindows()
        print("程序已退出")

if __name__ == "__main__":
    # 检查是否安装pyrealsense2
    try:
        import pyrealsense2
    except ImportError:
        print("请先安装pyrealsense2: pip install pyrealsense2")
        exit(1)
    
    # 运行程序
    handler = RealSenseHandler()
    handler.start()
    handler.run()
    