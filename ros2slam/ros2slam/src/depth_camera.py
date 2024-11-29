#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from openni import openni2
from openni import _openni2 as c_api

class DepthCamera(Node):
    def __init__(self):
        super().__init__('depth_camera')

        # Initialize OpenNI
        self.get_logger().info('Initializing OpenNI...')
        openni2.initialize('/home/sora/slam-arena4/ros2slam/resource/libs/')
        self.dev = openni2.Device.open_any()

        # Create the depth stream
        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.start()
        self.depth_stream.set_video_mode(c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
            resolutionX=640,
            resolutionY=480,
            fps=30
        ))

        # Initialize the ROS 2 publisher for depth image
        self.publisher_ = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        # Initialize cv_bridge to convert OpenCV image to ROS Image message
        self.bridge = CvBridge()

        # Create a timer to publish depth images at regular intervals (10Hz)
        self.timer = self.create_timer(0.1, self.publish_depth_image)

    def publish_depth_image(self):
        # Read a new frame from the depth stream
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()

        # Convert the frame to a numpy array and reshape
        img = np.frombuffer(frame_data, dtype=np.uint16)
        img.shape = (480, 640)

        # Optionally, create a 3-channel image for display purposes (not required for publishing)
        img_rgb = np.stack([img] * 3, axis=-1)  # Repeat the depth image to form a 3-channel image

        # Convert the numpy image to a ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding="bgr8")

        # Publish the image message
        self.publisher_.publish(img_msg)

        # Optionally, display the image using OpenCV for debugging
        cv2.imshow("Depth Image", img)
        cv2.waitKey(1)

    def destroy_node(self):
        # Stop the depth stream and clean up OpenNI
        self.depth_stream.stop()
        openni2.unload()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = DepthCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
