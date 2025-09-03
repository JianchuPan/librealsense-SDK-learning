import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from std_srvs.srv import Empty  # 添加这行导入


class RealSenseImageProcessor(Node):
    def __init__(self):
        super().__init__('realsense_image_processor')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建保存图像的目录
        self.save_dir = os.path.join(os.path.expanduser('~'), 'realsense_images/realsense_images')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"图像将保存到: {self.save_dir}")
        
        # 订阅彩色图像
        self.color_subscriber = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_image_callback,
            10
        )
        
        # 订阅深度图像
        self.depth_subscriber = self.create_subscription(
            Image,
            'camera/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )
        
        # 创建保存图像的服务（可选功能）
        self.save_image_service = self.create_service(
            Empty,
            'save_realsense_images',
            self.save_images_service_callback
        )
        
        # 存储最新的图像用于服务调用
        self.latest_color_image = None
        self.latest_depth_image = None
        
        self.get_logger().info("RealSense图像处理器已启动")

    def color_image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_color_image = cv_image
            
            # 显示图像
            cv2.imshow('Color Image', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"处理彩色图像出错: {str(e)}")

    def depth_image_callback(self, msg):
        try:
            # 转换深度图像
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
            
            # 处理深度图像以便显示
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # 显示深度图像
            cv2.imshow('Depth Image', depth_colormap)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"处理深度图像出错: {str(e)}")

    def save_images_service_callback(self, request, response):
        """服务回调：保存当前图像"""
        if self.latest_color_image is not None and self.latest_depth_image is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # 保存彩色图像
            color_path = os.path.join(self.save_dir, f"color_{timestamp}.png")
            cv2.imwrite(color_path, self.latest_color_image)
            
            # 保存深度图像（原始数据和彩色映射）
            depth_path = os.path.join(self.save_dir, f"depth_{timestamp}.png")
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.latest_depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
            cv2.imwrite(depth_path, depth_colormap)
            
            # 保存原始深度数据（可选）
            depth_raw_path = os.path.join(self.save_dir, f"depth_raw_{timestamp}.npy")
            import numpy as np
            np.save(depth_raw_path, self.latest_depth_image)
            
            self.get_logger().info(f"图像已保存: {color_path}, {depth_path}")
        else:
            self.get_logger().warn("没有可用图像可保存")
            
        return response

def main(args=None):
    # 导入Empty服务类型（需要放在这里避免循环导入）
    from std_srvs.srv import Empty
    
    rclpy.init(args=args)
    image_processor = RealSenseImageProcessor()
    
    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        image_processor.destroy_node()
        rclpy.shutdown()
    