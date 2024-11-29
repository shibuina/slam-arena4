#!/usr/bin/python
import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.publisher = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 fps
        
        # Initialize OpenNI
        openni2.initialize('/home/sora/slam-arena4/ros2slam/Astra-camera-installation-main/libs/')
        self.dev = openni2.Device.open_any()
        
        # Setup depth stream
        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.start()
        self.depth_stream.set_video_mode(c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
            resolutionX=640,
            resolutionY=480,
            fps=30))

    def timer_callback(self):
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()
        
        img = np.frombuffer(frame_data, dtype=np.uint16)
        img.shape = (480, 640)  # Reshape to 2D array
        
        # Convert to ROS message
        ros_msg = self.bridge.cv2_to_imgmsg(img, encoding="16UC1")
        ros_msg.header.frame_id = "camera_depth_optical_frame"
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher.publish(ros_msg)
    def __del__(self):
        if hasattr(self, 'depth_stream'):
            self.depth_stream.stop()
        if hasattr(self, 'dev'):
            self.dev.close()
        openni2.unload()
def main():
    rclpy.init()
    depth_publisher = DepthPublisher()
    
    try:
        rclpy.spin(depth_publisher)
    except KeyboardInterrupt:
        pass
    
    depth_publisher.depth_stream.stop()
    openni2.unload()
    depth_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()