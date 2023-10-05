#!/usr/bin/python3
import cv2, socket, pickle,  base64    # Import Modules
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

_NODE_NAME = "oc_node"
_PUB_TOPIC = "offsets"
_QUEUE_SIZE = 100
_PUBLISH_PERIOD_SEC = 0.01

class OffsetCalcNode(Node):
    def __init__(self, capture: cv2.VideoCapture, node_name: str =_NODE_NAME, pub_period: float=_PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)
        self.cap = capture       
        
        # define calculations publish topic
        self.publisher = self.create_publisher(Vector3Stamped, _PUB_TOPIC, _QUEUE_SIZE)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.calculate_offsets_callback)
        self.i = 0  

    def calculate_offsets_callback(self) -> None:
       
        if(self.cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            if ret == True:
                height, width = frame.shape[:2] # Get dimensions of frames
                
                # Display the resulting frame
                frame, yolo_out = self.boundingBox(frame, width, height)
                cv2.imshow('Frame',frame)

                proportion = self.getProportion(yolo_out[2], yolo_out[3], [width, height])

                msg = Vector3Stamped()
                msg.header.stamp = Node.get_clock(self).now().to_msg()
                msg.vector.x = float(yolo_out[0]) # x offset
                msg.vector.y = float(yolo_out[1]) # y offset
                msg.vector.z = float(proportion) # proportion of frame

                self.publisher.publish(msg)
                self.i += 1

    def boundingBox(self, image, w, h):

        yolo_out = [0, 0, 0, 0]

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
                yolo_out = [center[0]/w, center[1]/h, (x2-x1)/w, (y2-y1)/h]
                
        return image, yolo_out

    def get_rectangle_center(self, x1, y1, x2, y2):
        # Calculate the center coordinates
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        return [center_x, center_y]

    def getOffset(self, x_c, y_c):
        '''
        Calculates the offset from the center of the frame to the center of the object.
        inputs:
            x_c, y_c: normalized coordinates of the center of the object.
        returns:
            targetOffset: array with target offset in x and y direction from the center (scale in both axis from -1 to 1)
        '''

        targetOffset = [0,0] # initialize to zero

        # Convert value from scale [0, 1] to scale [-1, 1]
        targetOffset[0] = 2*x_c - 1
        targetOffset[1] = -(2*y_c - 1)

        return targetOffset
    
    def getProportion(self, w, h, frameDims):
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

        return proportion

       
def main():
    cap =  cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file") 
    
    rclpy.init(args=None)
    calc_publisher = OffsetCalcNode(capture=cap)

    rclpy.spin(calc_publisher)

    calc_publisher.destroy_node()
    rclpy.shutdown()
  
    # When everything done, release the video capture object and close all frames
    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
  main()