# Import the necessary libraries
import cv2
import numpy as np
import time
import sys
import imutils
from scipy.interpolate import interp1d
from imutils.video import VideoStream
import argparse
# import math

# Import user-defined RANSAC file
from ransac_function_linear import *

# Define the sliding window funcgion for scanning a frame
def sliding_window(image, stepX, stepY, windowSize):
    # Slide window left to right (positive x increments)
    for x in range(0, image.shape[1], stepX):
        # Slide window from top to bottom (positive y increments)
        for y in range(0, image.shape[0], stepY):
            # Yield the current window
            yield(x, y, image[y:y + windowSize[1], x:x + windowSize[0]])
            
# Create function for automatically computing canny lower and upper bounds
def auto_canny(image, sigma = 0.33):
    # Compute the median of the single channel (grayscale) pixel intensities
    v = np.median(image)
    
    # apply automatic Canny edge deteciton using computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    
    # return the edged image
    return edged

# Define the image processing function to be performed on each frame
# after sizing and orienting
def frame_process(image, color_floor, color_ceiling):
    # dimensions of resized image (width, height) are now (640, 360)
    # Convert image to HSV colorspace
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, color_floor, color_ceiling)
    mask = cv2.dilate(mask, None, iterations=1)
    mask = cv2.erode(mask, None, iterations=1)
    
    # Use the Automatic Canny Edge Detection function
    auto = auto_canny(mask)

#    # Calculate sobel gradients of grayscale image in horizontal
#    # then vertical directions
#    sobelX = cv2.Sobel(mask, cv2.CV_64F, 1, 0)
#    sobelY = cv2.Sobel(mask, cv2.CV_64F, 0, 1)
##
##    # Convert the 64-bit floating point data to unsigned 8-bit integers for processing
#    sobelX = np.uint8(np.absolute(sobelX))
#    sobelY = np.uint8(np.absolute(sobelY))
##
##    # Combine the horizontal and vertical edge-gradient data
#    sobelCombined = cv2.bitwise_or(sobelX, sobelY)

    # Threshold the canny-edged image and have the function return the result
    (T, thresh) = cv2.threshold(auto, 100, 255, cv2.THRESH_BINARY)
    return thresh

# Define perspective transform function
def perspective_warp(img, width, height):
    # Boundary points for Region of Interest (ROI)
    points = np.float32([tl, tr, br, bl])
    # Define corner post-warp locations based on desired resulting ratio (3:5 index card ratio)
    src = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(points, src)
    # Apply perspective warp, final image will be 300x500 pixels
    warp = cv2.warpPerspective(img, matrix, (width, height))
    return warp

# Define inverse perspective transform function
def inv_perspective_warp(img, width, height):
    src = np.float32([tl, tr, br, bl])
    # Define corner post-warp locations based on desired resulting ratio (3:5 index card ratio for prototyping)
    points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(points, src)
    # Apply perspective warp, final image will be 300x500 pixels
    inv_warp = cv2.warpPerspective(img, matrix, (width, height))
    return inv_warp

# Supply path to video file
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())

# If no video file path is provided in the argument parsing
if not args.get("video", False):
    # For a webcam use VideoStream
    vs = VideoStream(src=0).start()

    # For the picamera specify use of the PiCamera and define resolution
#     vs = VideoStream(usePiCamera=True, resolution=(672, 480)).start()
# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# Prototyping code for efficient ROI calculations
# Define the tilt angle (radians) of the camera with respect to vertical axis
# Example angle 40 degrees or pi/4.5
# alpha_tilt = math.pi/4.5
# Define the vertical height of the Region of Interest (ROI) window
# roi_height = 380

# Define the warped ROI (Region of Interest) upper base width
top_w = 220
# Define the top horizontal boundary of the warped ROI, with respect to [0,0]
top_h = 100

# Define the frame dimensions of the warped image
warp_w = 640
warp_h = 480
# Define sliding window size for warpped image
warp_winW = warp_w // 8
warp_winH = warp_h // 16

# Define mid-image refernce for warped frame
mid_ref = warp_w//2
# Alternate variable for differentating right and left lanes based
# on both vertical and horizontal centroid
#mid_ref = sqrt(abs((warp_w//2) ** 2 - (warp_h//2) ** 2))

# Define functions for clean terminal print
cursor_up = '\x1b[1A'
erase_line = '\x1b[2K'


while True:
    # To reference a specific key press
    key = cv2.waitKey(1) & 0xFF
    # Define colors to be used later
    green = (0, 255, 0)
    white = (255, 255, 255)
    red = (0, 0, 255)
    blue = (255, 0, 0)
    

    # Grab the current frame
    image = vs.read()
    # handle the frame from VideoCapture or VideoStream
#    image = image[1] if args.get("video", False) else image

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if image is None:
        break

    # If using the Picamera, rotate the image 180 degrees 
#     frame = imutils.rotate_bound(image, 180)

    # If using a webcam, specify frame width 
    # Example resolution in use: [480, 640]
    # Note: the width/height aspect ratio of my webcam is 1.333333
    resized = imutils.resize(image, width = 640)
    width = resized.shape[1]
    height = resized.shape[0]
    
    # Prototyping code for efficient ROI calculation
#     roi_lower_width = width
#     roi_height = height - top_h
#     roi_upper_width = int(roi_lower_width - 2*(roi_height/math.tan(alpha_tilt)))
#     print("ROI upper width: {}".format(roi_upper_width))
    # Define Region of Interest (ROI) verticies for perspective transform [x,y]
    bl = [0, height]
    br = [width, height]
    tl = [width//2 - top_w//2, top_h]
    tr = [width//2 + top_w//2, top_h]
        
    # Call the frame processing function
    edged = frame_process(resized, color_floor = (60, 60, 60), color_ceiling = (90, 255, 255))

 # Initialize arrays for storing potential points
    warp_y_inds = []
    warp_x_inds = []
       
    # Perform perspective transform   
    warped = perspective_warp(edged, warp_w, warp_h)
    warp_ransaced = perspective_warp(resized, warp_w, warp_h)
    # Create a blank-canvas copy of the warped canny-edged image, and the warped original image
    warped_potential = np.zeros_like(warped)
    warp_lanes = np.zeros_like(warp_ransaced)
               
    # Employ sliding window algorithm on warped image (birds-eye view)
    for (x,y, window) in sliding_window(warped, stepX = warp_winW, stepY = warp_winH, windowSize=(warp_winW, warp_winH)):
         # If the window is not the proper size ignore it
        if window.shape[0] != warp_winH or window.shape[1] != warp_winW:
            continue
        # Isolate portion of image within window
        crop = warped[y:y + warp_winH, x:x + warp_winW]
        nonzero = crop.nonzero()
        # If no white pixels exist in bouding box, skip it
        if len(nonzero[0]) == 0:
            continue
        
        # Otherwise record (x,y) coordinates of average distribution of white
        nonzeroX = np.array(nonzero[1])
        nonzeroY = np.array(nonzero[0])
        averageY = np.int(np.mean(nonzeroY)) + y
        averageX = np.int(np.mean(nonzeroX)) + x
        # Append the new coordinates to the running list of points
        warp_y_inds.append(averageY)
        warp_x_inds.append(averageX)
        
    if len(warp_x_inds) > 3:
#        print("Number of potential points: {}".format(len(warp_x_inds)))
        # Loop through the average location points and plot them on the image
        for i in range(0, len(warp_x_inds), 1):
            cv2.circle(warped_potential, (warp_x_inds[i], warp_y_inds[i]), 2, white, -1)

        # Convert the list of x-indicies and y-indicies into an ordered pair list
        avg_points = list(zip(warp_x_inds, warp_y_inds))
        lane_1 = []
        
        # Apply the RANSAC algorithm to the full list of averaged points
        lane_1 = compute(avg_points, 30, 250, 0.2)
        # Remove the RANSAC-verified lane points from the list of potential points
        if lane_1 != None:
            for i in range(0, len(lane_1[0]), 1):
                avg_points.remove((lane_1[0][i], lane_1[1][i]))
                
            # Determine number of ordered pairs within list         
            length = len(lane_1[0])
            # Extract arrays for the x-coordinates and the y-coordinates separately
            lane_1x = lane_1[0]
            lane_1y = lane_1[1]
            
            # Compute the average location of lane_1 points
            avg_1x = np.int(np.mean(lane_1x))
            avg_1y = np.int(np.mean(lane_1y))
            avg_position1 = sqrt(abs(avg_1x ** 2 - avg_1y ** 2))
            
            # If the average position of the lane is greater than the center pixel
            # the lane is likely a "right" lane, draw it in red
            if avg_1x >= mid_ref:
                color1 = red
                
            # Otherwise, it's likely a left lane, draw it in blue
            else:
                color1 = blue
            
            # Plot the left RANSAC points in the appropriate color
            for i in range(0, length, 1):
                cv2.circle(warp_ransaced, (lane_1x[i], lane_1y[i]), 2, color1, -1)

            # Generate second-degree best-fit curve as a function of Y
            # so that the curve can easily be bounded vertically                
            poly_1 = np.polyfit(lane_1y, lane_1x, 2)
            lane_1_function = np.poly1d(poly_1)
            min_lane_1 = min(lane_1y)
            max_lane_1 = max(lane_1y)
                
            # Generate a list of ordered pairs for each lane based on
            # best-fit curve function, ensuring the the coordinates
            # are integer data types
            lane_1_plotX = []
            lane_1_plotY = []
            for i in range(min_lane_1 - 5, max_lane_1 + 5, 5):
                lane_1_plotY.append(i)
                x = int(lane_1_function(i))
                lane_1_plotX.append(x)

            # Combine the x and y coordinates of the lane into
            # a single list of ordered pairs
            # Lane 1 list
            lane_1_points = list(zip(lane_1_plotX, lane_1_plotY))
                        
            # Then plot the Lane 1 line-approximation on the warped image
            cv2.polylines(warp_lanes, np.int32([lane_1_points]), False, color1, 8)
            
        # If there are more points for a second lane
        if avg_points != None:    
            lane_2 = []
            lane_2 = compute(avg_points, 30, 250, 0.3)
        
            if lane_2 != None:
                length = len(lane_2[0])
                # Extract an array for the x-coordinates and the y-coordinates separately
                lane_2x = lane_2[0]
                lane_2y = lane_2[1]
                # Compute the average location of lane_1 points
                avg_2x = np.int(np.mean(lane_2x))
                avg_2y = np.int(np.mean(lane_2y))
                avg_position2 = sqrt(abs(avg_2x ** 2 - avg_2y ** 2))
            
                # If the average position of the lane is greater than the center pixel
                # the lane is likely a "right" lane, draw it in red
                if avg_2x >= mid_ref:
                    color2 = red
                
                # Otherwise, it's likely a left lane, draw it in blue
                else:
                    color2 = blue
                        
                # Plot the left RANSAC points in the appropriate color
                for i in range(0, length, 1):
                    cv2.circle(warp_ransaced, (lane_2x[i], lane_2y[i]), 2, color2, -1)

                # Generate second-degree best-fit curve as a function of Y
                # so that the curve can easily be bounded vertically                
                poly_2 = np.polyfit(lane_2y, lane_2x, 2)
                lane_2_function = np.poly1d(poly_2)
                min_lane_2 = min(lane_2y)
                max_lane_2 = max(lane_2y)
                
                # Generate a list of ordered pairs for each lane based on
                # best-fit curve function, ensuring the the coordinates
                # are integer data types
                lane_2_plotX = []
                lane_2_plotY = []
                for i in range(min_lane_2 - 5, max_lane_2 + 5, 5):
                    lane_2_plotY.append(i)
                    x = int(lane_2_function(i))
                    lane_2_plotX.append(x)

                # Combine the x and y coordinates of the lane into
                # a single list of ordered pairs
                # Lane 2 list
                lane_2_points = list(zip(lane_2_plotX, lane_2_plotY))
                        
                # Then plot the Lane 2 line-approximation on on the warped image
                cv2.polylines(warp_lanes, np.int32([lane_2_points]), False, color2, 8)
            
            # Perform inverse persective transform of blank canvas with lanes drawn
            # to obtain original view
            unwarped = inv_perspective_warp(warp_lanes, width, height)
             
            # Apply the drawn left and right lanes onto original image "resized"   
            resized = cv2.addWeighted(resized, 1, unwarped, 1, 0)
            # Clean print by writing over previous message
            sys.stdout.write(cursor_up)
            sys.stdout.write(erase_line)
            print("Lanes detected")
        # Otherwise, less than 1 left AND right lane points were found
        else:
            # Clean print by writing over previous message
            sys.stdout.write(cursor_up)
            sys.stdout.write(erase_line)
            print("Only one curve found")
            
    # Otherwise, not enough sample points were found (less than 4 in total)
    else:
        # Clean print by writing over previous message
         sys.stdout.write(cursor_up)
         sys.stdout.write(erase_line)
         print("Insufficient data points")
         
    # Convert the ROI boundary points to tuple data types for line-drawing 
    vertexA = tuple(bl)
    vertexB = tuple(br)
    vertexC = tuple(tr)
    vertexD = tuple(tl)
    # Draw outline of ROI in bright green on original image for viewer reference
    cv2.line(resized, vertexA, vertexB, green, 2)
    cv2.line(resized, vertexB, vertexC, green, 2)
    cv2.line(resized, vertexC, vertexD, green, 2)
    cv2.line(resized, vertexD, vertexA, green, 2)

    # Display the lane detection results    
    cv2.imshow("Detected Lanes", resized)
    cv2.imshow("Edges", edged)
    cv2.imshow("Warped Potential Points", warped_potential)
    cv2.imshow("Ransaced Points", warp_ransaced)

    # Quit the program if the 'q' key is pressed  
    if key == ord("q"):
        print("Program terminated by user")
        break

# Close all windows and stop the video stream
cv2.destroyAllWindows()
vs.stop()
