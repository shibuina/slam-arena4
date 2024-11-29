#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from openni import openni2
from openni import _openni2 as c_api

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        
        # Initialize OpenNI depth device
        openni2.initialize('/home/sora/slam-arena4/ros2slam/Astra-camera-installation-main/libs/')
        self.dev = openni2.Device.open_any()
        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.start()
        self.depth_stream.set_video_mode(c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM,
            resolutionX=640,
            resolutionY=480,
            fps=30
        ))

        # Publisher for depth images
        self.publisher_ = self.create_publisher(Image, 'depth_image', 10)
        self.bridge = CvBridge()

        # Timer to publish depth images at regular intervals
        self.timer = self.create_timer(0.1, self.publish_depth_image)

        # For mouse event to select region of interest
        self.refPt = []
        self.selecting = False
        cv2.namedWindow("Depth Image")
        cv2.setMouseCallback("Depth Image", self.point_and_shoot)

    def point_and_shoot(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x, y)]
            self.selecting = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x, y))
            self.selecting = False

    def publish_depth_image(self):
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()

        # Convert depth frame to numpy array
        img = np.frombuffer(frame_data, dtype=np.uint16)
        img.shape = (1, 480, 640)
        img = np.concatenate((img, img, img), axis=0)
        img = np.swapaxes(img, 0, 2)
        img = np.swapaxes(img, 0, 1)

        if len(self.refPt) > 1:
            img = img.copy()
            cv2.rectangle(img, self.refPt[0], self.refPt[1], (0, 255, 0), 2)

        # Convert numpy image to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono16")
        
        # Publish the image
        self.publisher_.publish(img_msg)

        # Display the image for debugging
        cv2.imshow("Depth Image", img)
        cv2.waitKey(1)

    def destroy_node(self):
        self.depth_stream.stop()
        openni2.unload()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
