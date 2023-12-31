#!/usr/bin/python3
"""
This Program is for Voice less video Streaming using UDP protocol 
-------
This is a Server
    - Run Client Code first then run Server Code
    - Configure IP and PORT number of your Server
    
Requirements
    - Outside Connection 
    - IP4v needed select unused port number
    - WebCamera needed
"""
import cv2  # Import Modules
import numpy as np
import os

# from ultralytics import YOLO
from pathlib import Path
from ultralytics import YOLO

def throughVideo():
  video_file_name = "football_video.mp4"
  base_path = os.path.abspath(os.path.dirname(__file__))
  relative_path_video = "/src/target_offset/target_offset/videos/" + video_file_name
  
  # Load video and YOLO model
  video = base_path + relative_path_video
  model = YOLO(base_path + "/src/target_offset/target_offset/ball_weights.pt")

  cap = cv2.VideoCapture(video)

  # Check if camera opened successfully
  if (cap.isOpened()== False): 
    print("Error opening video stream or file")
  
  # Read until video is completed
  while(cap.isOpened()):
    
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:
      height, width = frame.shape[:2] # Get dimensions of frames
      
      # Display the resulting frame
      frame, yolo_out = boundingBoxYOLO(model, frame, width, height)
      # frame, yolo_out = boundingBox(frame, width, height)
      imS = cv2.resize(frame, (960, 540))
      cv2.imshow('Ball Detection',imS)

      if yolo_out != None:
        offsets = getOffset(yolo_out[0], yolo_out[1])
        proportion = getProportion(yolo_out[2], yolo_out[3], [width, height])
        print("x-offset = {0}, y-offset = {1}, proportion = {2}".format(offsets[0],offsets[1],proportion))
  
      # Press Q on keyboard to  exit
      if cv2.waitKey(25) & 0xFF == ord('q'):
        break
  
    # Break the loop
    else: 
      break
  
  # When everything done, release the video capture object
  cap.release()
  
  # Closes all the frames
  cv2.destroyAllWindows()


def throughCameraStream():

  
  cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
  
  base_path = os.path.abspath(os.path.dirname(__file__))
  # model = YOLO(str(Path(base_path)) + "/src/target_offset/target_offset/ball_weights.pt")  

  # Check if camera opened successfully
  if (cap.isOpened()== False): 
    print("Error opening video stream or file")
  
  # Read until video is completed
  while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:
      height, width = frame.shape[:2] # Get dimensions of frames
      
      # Display the resulting frame
      # frame, yolo_out = boundingBoxYOLO(model, frame, width, height)
      frame, yolo_out = boundingBox(frame, width, height)
      cv2.imshow('Ball Detection',frame)

      if yolo_out != None:
        offsets = getOffset(yolo_out[0], yolo_out[1])
        proportion = getProportion(yolo_out[2], yolo_out[3], [width, height])
        print("x-offset = {0}, y-offset = {1}, proportion = {2}".format(offsets[0],offsets[1],proportion))
  
      # Press Q on keyboard to  exit
      if cv2.waitKey(25) & 0xFF == ord('q'):
        break
  
    # Break the loop
    else: 
      break
  
  # When everything done, release the video capture object
  cap.release()
  
  # Closes all the frames
  cv2.destroyAllWindows()

def boundingBoxYOLO(model, image, w, h):
    '''
    https://dipankarmedh1.medium.com/real-time-object-detection-with-yolo-and-webcam-enhancing-your-computer-vision-skills-861b97c78993
    '''
    yolo_out = None

    results = model(image, stream=True)

    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Get bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to intsqq

            # Draw the bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 5)
            center = get_rectangle_center(x1, y1, x2, y2)
            yolo_out = [center[0]/w, center[1]/h, (x2-x1)/w, (y2-y1)/h]
    
    return image, yolo_out    

def boundingBox(image, w, h):

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
          center = get_rectangle_center(x1, y1, x2, y2)
          # center = [center[0]/w, center[1]/h]
          # dims_scaled = [(x2-x1)/w, (y2-y1)/h]
          yolo_out = [center[0]/w, center[1]/h, (x2-x1)/w, (y2-y1)/h]
          

  return image, yolo_out

def get_rectangle_center(x1, y1, x2, y2):
    # Calculate the center coordinates
    center_x = (x1 + x2) // 2
    center_y = (y1 + y2) // 2
    return [center_x, center_y]

def getOffset(x_c, y_c):
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
  
def getProportion(w, h, frameDims):
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

if __name__ == "__main__":
  # throughCameraStream()
  throughVideo()