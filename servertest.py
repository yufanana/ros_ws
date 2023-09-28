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
import cv2, socket, pickle,  base64    # Import Modules
import numpy as np

def main():

  cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

  # Check if camera opened successfully
  if (cap.isOpened()== False): 
    print("Error opening video stream or file")
  
  # Read until video is completed
  while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:
  
      # Display the resulting frame
      frame = boundingBox(frame)
      cv2.imshow('Frame',frame)
  
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

def boundingBox(image):

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

  return image

if __name__ == "__main__":
  main()