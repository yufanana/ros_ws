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
import cv2, socket, numpy, pickle,  base64    # Import Modules

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

'''
# AF_INET refers to the address of family of ip4v
# SOCK_DGRAM means connection oriented UDP protocol
s=socket.socket(socket.AF_INET , socket.SOCK_DGRAM)  # Gives UDP protocol to follow
ip="127.0.0.1"   # Server public IP
port=5600            # Server Port Number to identify the process that needs to recieve or send packets
s.bind((ip,port))     # Bind the IP:port to connect 

# In order to iterate over block of code as long as test expression is true
while True:
    x, _ = s.recvfrom(700000)    # Recieve byte code sent by client using recvfrom
    data = base64.b64decode(x)
    npdata = numpy.frombuffer(data, dtype=numpy.uint8)
    frame = cv2.imdecode(npdata,1)
    print('frame',frame)
    cv2.imshow('stream', frame)
    if cv2.waitKey(10) == 13:  # Press Enter then window will close
        break
    
    # clientip = x[1][0]         # x[1][0] in this client details stored,x[0][0] Client message Stored
    # data=x[0]                  # Data sent by client
    # data=pickle.loads(data)    # All byte code is converted to Numpy Code 
    # data = cv2.imdecode(data, cv2.IMREAD_COLOR)  # Decode 
    # cv2.imshow('my pic', data) # Show Video/Stream
    # if cv2.waitKey(10) == 13:  # Press Enter then window will close
    #     break
cv2.destroyAllWindows()        # Close all windows
'''