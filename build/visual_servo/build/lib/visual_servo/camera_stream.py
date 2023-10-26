#!/usr/bin/python3

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    # QoSDurabilityPolicy
# from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

_CAM_NODE_NAME = "camera_node"
_CAM_PUB_TOPIC = "camera_stream"
_CAM_FRAME_ID = "Camera_Capture"
_QUEUE_SIZE = 100
_PUBLISH_PERIOD_SEC = 0.01

class CameraStreamNode(Node):

    def __init__(self, capture: cv2.VideoCapture) -> None:
        super().__init__(_CAM_NODE_NAME)
        self.bridge = CvBridge()
        self.cap = capture
        self.pub = self.create_publisher(Image, _CAM_PUB_TOPIC, _QUEUE_SIZE)
        
        # define publishing frequency and callback function
        self.timer_ = self.create_timer(_PUBLISH_PERIOD_SEC, self.capture_image_callback)
        self.i = 0

    def capture_image_callback(self) -> None:
        """
        Captures an image from the camera via RTSP and publishes it as a ROS Image message.
        """
        _, frame = self.cap.read()
        shape = np.shape(frame)

        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = _CAM_FRAME_ID
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.height = shape[0]
        msg.width = shape[1]
        msg.step = shape[2]*shape[1]

        self.pub.publish(msg)
        self.i += 1

def main(args=None):
    capture = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
    
    rclpy.init(args=args)
    camera_publisher = CameraStreamNode(capture=capture)

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass

    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()

if __name__ == "__main__":
    main()
