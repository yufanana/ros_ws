import cv2
import numpy as np

# Load the input image
input_image = "/home/oda/Downloads/ball.png"  # Replace with your input image path
image = cv2.imread(input_image)

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
        minRadius=100,  # Adjust this parameter for the minimum circle radius
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
