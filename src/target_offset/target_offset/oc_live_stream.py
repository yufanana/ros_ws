#!/usr/bin/python3
import cv2
import numpy as np
import rclpy
import os

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from ultralytics import YOLO
from pathlib import Path
from typing import Tuple, List


class OffsetCalcVideoStreamNode(Node):
    def __init__(self, classes=[32]) -> None:

        # Initialize constants
        _NODE_NAME = "oc_node"
        _PUB_TOPIC = "/visual_servo/target_offset"
        _SUB_TOPIC = "/color/image_raw"
        _QUEUE_SIZE = 10
        _PUBLISH_PERIOD_SEC = 0.01
        super().__init__(_NODE_NAME)

        # Initialize QoSProfile to make published messages match with the PX4
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Initialize model and tracker
        self.__base_path = os.path.abspath(os.path.dirname(__file__))
        self.__relative_path_model = "/weights/ball_weights.pt"
        self.__model = YOLO(self.__base_path  + self.__relative_path_model)
        # self.__tracker = self.__model.track(source=source, classes=classes, conf=0.3, show=False, stream=True)

        # Ball ID
        self.__ball_ID = None
        
        # define calculations publish topic
        self.__offset_publisher = self.create_publisher(Vector3Stamped, _PUB_TOPIC, self.qos_profile)

        # define video stream publish topic
        self.__video_frames_publisher = self.create_publisher(Image, 'video_stream', _QUEUE_SIZE)
        self.__br = CvBridge()

        # Create image subscriber
        self.__subscription = self.create_subscription(Image, _SUB_TOPIC, self.listener_callback, 10)
        self.__subscription # prevent unused variable warning

        # define publishing frequency and callback function
        # self.__offset_timer = self.create_timer(_PUBLISH_PERIOD_SEC, self.calculate_offsets_callback)
        # self.__listener_timer = self.create_timer(_PUBLISH_PERIOD_SEC, self.listener_callback)
        self.i = 0  

    def listener_callback(self, msg: Image) -> None:
        """
        Callback function.
        """
    
        # Convert ROS Image message to OpenCV image
        frame = self.__br.imgmsg_to_cv2(msg, 'bgr8')
        
        if frame is not None:
            height, width = frame.shape[:2] # Get dimension of frame
            
            # Display the resulting frame
            frame, yolo_out = self.boundingBoxYOLO(frame, width, height)
            imS = cv2.resize(frame, (960, 540))

            # Publish image with bounding boxes
            self.__video_frames_publisher.publish(self.__br.cv2_to_imgmsg(imS))

            # Calculate and publish offsets
            if yolo_out != None:
                offsets = self.getOffset(yolo_out[0], yolo_out[1])
                proportion = self.getProportion(yolo_out[2], yolo_out[3], [width, height])

                msg = Vector3Stamped()
                msg.header.stamp = Node.get_clock(self).now().to_msg()
                msg.vector.x = offsets[0] # x offset
                msg.vector.y = offsets[1] # y offset
                msg.vector.z = proportion # proportion of frame

                self.__offset_publisher.publish(msg)

                print('yolo_out: ', yolo_out)

            self.i += 1        
        

    def calculate_offsets_callback(self) -> None:
       
        for result in self.__tracker:
            # Capture frame-by-frame
            frame = result.orig_img
            if frame:
                height, width = frame.shape[:2] # Get dimensions of frames
                
                # Display the resulting frame
                frame, yolo_out = self.boundingBoxYOLO(frame, width, height)
                # frame, yolo_out = self.boundingBox(frame, width, height)
                imS = cv2.resize(frame, (960, 540))
                # cv2.imshow('Ball Detection',imS)
                self.__video_frames_publisher.publish(self.__br.cv2_to_imgmsg(imS))

                if yolo_out != None:
                    offsets = self.getOffset(yolo_out[0], yolo_out[1])
                    proportion = self.getProportion(yolo_out[2], yolo_out[3], [width, height])

                    msg = Vector3Stamped()
                    msg.header.stamp = Node.get_clock(self).now().to_msg()
                    msg.vector.x = offsets[0] # x offset
                    msg.vector.y = offsets[1] # y offset
                    msg.vector.z = proportion # proportion of frame

                    self.__offset_publisher.publish(msg)

                    print('yolo_out: ', yolo_out)
                self.i += 1
            else:
                self.__cap.release()
                self.destroy_node()
                rclpy.shutdown()
                cv2.destroyAllWindows()
        
    def boundingBoxYOLO(self, image, w, h) -> Tuple[np.ndarray, List[float]]:

        yolo_out = None

        results = self.__model(image, stream=True, conf=0.35)

        for r in results:
            boxes = r.boxes

            for box in boxes:
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to ints

                # Draw the bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                center = self.get_rectangle_center(x1, y1, x2, y2)
                yolo_out = [center[0]/w, center[1]/h, (x2-x1)/w, (y2-y1)/h]
        
        return image, yolo_out
        

    def boundingBox(self, image, w, h) -> Tuple[np.ndarray, List[float]]:

        yolo_out = None

        # Convert the image to grayscale for better edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles using the Hough Circle Transform
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,  # Adjust this parameter to control circle separation
            param1=100,  # Adjust this parameter to control edge detection
            param2=50,   # Adjust this parameter to control circle detection sensitivity
            minRadius=10,  # Adjust this parameter for the minimum circle radius
            maxRadius=500   # Adjust this parameter for the maximum circle radius
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            
            for circle in circles[0, :]:
                # Get the center and radius of the circle
                center_x, center_y, radius = circle[0], circle[1], circle[2]
                
                # Calculate the bounding box coordinates
                x1, y1 = center_x - radius, center_y - radius
                x2, y2 = center_x + radius, center_y + radius
                
                # Draw the bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                center = self.get_rectangle_center(x1, y1, x2, y2)
                yolo_out = [float(center[0]/w), float(center[1]/h), float((x2-x1)/w), float((y2-y1)/h)]
                
        return image, yolo_out

    def get_rectangle_center(self, x1, y1, x2, y2) -> List[float]:
        # Calculate the center coordinates
        center_x = float((x1 + x2) // 2)
        center_y = float((y1 + y2) // 2)
        return [center_x, center_y]

    def getOffset(self, x_c, y_c) -> List[float]:
        '''
        Calculates the offset from the center of the frame to the center of the object.
        inputs:
            x_c, y_c: normalized coordinates of the center of the object.
        returns:
            targetOffset: array with target offset in x and y direction from the center (scale in both axis from -1 to 1)
        '''

        targetOffset = [0,0] # initialize to zero

        # Convert value from scale [0, 1] to scale [-1, 1]
        targetOffset[0] = float(2*x_c - 1)
        targetOffset[1] = float(-(2*y_c - 1))

        return targetOffset
    
    def getProportion(self, w, h, frameDims) -> float:
        '''
        Calculates the oproportion of the frame that the bounding box takes up.
        inputs:
            w, h:       normalized width and height of the frame.
            frameDims:  array with the width and height [w, h] of the picture frame.
        returns:
            proportion: the proportion of the fram that the bounding box fills out.
        '''

        w_pixels = w*frameDims[0]
        h_pixels = h*frameDims[1]

        proportion = w_pixels*h_pixels/(frameDims[0]*frameDims[1])*100

        return float(proportion)

       
def main():
  
    rclpy.init(args=None)
    calc_publisher = OffsetCalcVideoStreamNode()

    rclpy.spin(calc_publisher)

    calc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()