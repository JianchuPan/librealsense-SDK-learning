import pyrealsense2 as rs

# 尝试初始化相机管道，验证安装是否成功
try:
    pipeline = rs.pipeline()
    config = rs.config()
    # 尝试启动默认配置（不指定具体分辨率，仅验证SDK是否可用）
    pipeline.start(config)
    print("pyrealsense2 安装成功，可正常连接相机")
    pipeline.stop()
except Exception as e:
    print("pyrealsense2 安装或相机连接存在问题：", str(e))