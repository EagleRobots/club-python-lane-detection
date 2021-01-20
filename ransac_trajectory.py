# Developed: Emmanuel Jefferson
# Update: 1/13/2021
# Club file

# Import the necessary computer vision libraries
import cv2
import numpy as np
from time import sleep
import sys
import imutils
from scipy.interpolate import interp1d
from imutils.video import VideoStream
import argparse
import math
import subprocess

# Import user-defined RANSAC file
from ransac_function_linear import *

# Import ROS libraries
# from time import sleep
import rospy
import os
# Use data type UInt16, Float64, or string
from std_msgs.msg import Int16
# from std_msgs.msg import UInt16
# from std_msgs.msg import Float64
# from std_msgs.msg import String

# Launch ROS master node 'roscore'
roscore = subprocess.Popen('roscore')
# To launch ROS serial node, enter in separate terminal:
# rosrun rosserial_python serial_node.py /dev/ttyACM0

# Define the warped ROI (Region of Interest) upper base width
TOP_W = 220
# Define the top horizontal boundary of the warped ROI, with respect to [0,0] in top left corner
TOP_H = 100
# Define the frame dimensions of the warped image
WARP_W = 640
WARP_H = 480
# Define number of horizontal and vertical windows for sliding window algorithm
HORIZONTAL_WINDOWS = 4
VERTICAL_WINDOWS = 16
# Define sliding window size for warpped image
WARP_WinW = WARP_W // HORIZONTAL_WINDOWS
WARP_WinH = WARP_H // VERTICAL_WINDOWS
# Define mid-image refernce for warped frame
mid_ref = WARP_W//2
# Alternate variable for differentating right and left lanes based
# on both vertical and horizontal centroid
#mid_ref = sqrt(abs((WARP_W//2) ** 2 - (WARP_H//2) ** 2))

# Define RGB colors to be used later
green = (0, 255, 0)
white = (255, 255, 255)
red = (0, 0, 255)
blue = (255, 0, 0)

# Define operating system references for clean terminal print
CURSOR_UP = '\x1b[1A'
ERASE_LINE = '\x1b[2K'

# Define the sliding window funcgion for scanning a frame
def sliding_window(image, stepX, stepY, windowSize):
    # Slide window left to right (positive x increments)
    for x in range(0, image.shape[1], stepX):
        # Slide window from top to bottom (positive y increments)
        for y in range(0, image.shape[0], stepY):
            # Yield the current window
            yield(x, y, image[y:y + windowSize[1], x:x + windowSize[0]])
            
# Create function for automatically computing Canny lower and upper bounds
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
    
    # Apply color and clean-up functions
    mask = cv2.inRange(hsv, color_floor, color_ceiling)
    mask = cv2.dilate(mask, None, iterations=1)
    mask = cv2.erode(mask, None, iterations=1)
    
    # Use the Automatic Canny Edge Detection function
    auto = auto_canny(mask)

    # Threshold the canny-edged image and have the function return the result
    (T, thresh) = cv2.threshold(auto, 100, 255, cv2.THRESH_BINARY)
    return thresh

# Define perspective transform function
def perspective_warp(img, width, height):
    # Boundary points for Region of Interest (ROI)
    points = np.float32([roi_tl, roi_tr, roi_br, roi_bl])
    # Define corner post-warp locations based on desired resulting ratio (3:5 index card ratio)
    src = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(points, src)
    # Apply perspective warp, final image will be "width x height" pixels
    warp = cv2.warpPerspective(img, matrix, (width, height))
    return warp

# Define inverse perspective transform function
def inv_perspective_warp(img, width, height):
    src = np.float32([roi_tl, roi_tr, roi_br, roi_bl])
    # Define corner post-warp locations based on desired resulting ratio (3:5 index card ratio for prototyping)
    points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(points, src)
    # Apply perspective warp, final image will be "width x height" pixels
    inv_warp = cv2.warpPerspective(img, matrix, (width, height))
    return inv_warp

# Define lane computation function
def compute_lane(potential_points, min_dist, max_iterations, ransac_ratio):
    lane_points = []    
    # Apply the RANSAC algorithm to the full list of averaged points
    lane_points = compute_ransac(potential_points, min_dist, max_iterations, ransac_ratio)
    # Remove the RANSAC-verified inliers from the list of potential points
    if lane_points != None:
        for i in range(0, len(lane_points[0]), 1):
            potential_points.remove((lane_points[0][i], lane_points[1][i]))
 
    # Return the outliers and inliers separately
    return potential_points, lane_points

# Define function for drawing the computed lane
def draw_lane(frame, lane_points, red, blue):
    # Initialize default parameters for calculating lane heading
    top_x = 0
    top_y = 0
    if lane_points != None:
        # Determine number of ordered pairs within list         
        length = len(lane_points[0])
        # Extract arrays for the x-coordinates and the y-coordinates separately
        lane_x = lane_points[0]
        lane_y = lane_points[1]
        # Compute the average location of lane_1 points
        avg_x = np.int(np.mean(lane_x))
#         avg_y = np.int(np.mean(lane_y))
        
        # If the average position of the lane is greater than the center pixel
        # the lane is likely a "right" lane, draw it in red
        if avg_x >= mid_ref:
            color = red            
        # Otherwise, it's likely a left lane, draw it in blue
        else:
            color = blue        
        # Plot the RANSAC points in the appropriate color
        for i in range(0, length, 1):
            cv2.circle(warp_ransaced, (lane_x[i], lane_y[i]), 2, color, -1)

        # Generate second-degree best-fit curve as a function of Y
        # so that the curve can easily be bounded vertically                
        poly = np.polyfit(lane_y, lane_x, 2)
        lane_function = np.poly1d(poly)
        min_lane = min(lane_y)
        max_lane = max(lane_y)
        
        # Calculate parameters needed for  steering heading
        top_x = int(lane_function(min_lane))
        top_y = min_lane
        
        # Generate a list of ordered pairs for each lane based on
        # best-fit curve function, ensuring the the coordinates
        # are integer data types
        lane_plotX = []
        lane_plotY = []
        for i in range(min_lane - 5, max_lane + 5, 5):
            lane_plotY.append(i)
            x = int(lane_function(i))
            lane_plotX.append(x)            

        # Combine the x and y coordinates of the lane into
        # a single list of ordered pairs
        lane_points = list(zip(lane_plotX, lane_plotY))                    
        # Plot the Lane line-approximation on the warped image
        cv2.polylines(frame, np.int32([lane_points]), False, color, 8)
#     else:
#         print("Missing lane")
    else:
        print("RANSAC results unsatisfactory")
        sys.stdout.write(CURSOR_UP)
        sys.stdout.write(ERASE_LINE)
    return frame, top_x, top_y

# Define function for initializing ROS message publishing
def ros_talker(angle):
    # Define ROS publisher for sending data to topic 'steering'
    pub1 = rospy.Publisher('steering', Int16, queue_size=10)
#     pub2 = rospy.Publisher('status', String, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
#    rate = rospy.Rate(10)

      # Display to ROS console  
#     rospy.get_time()
#     rospy.loginfo(angle)
#    rospy.loginfo(position)
    
#     # Clean print by writing over previous message
#     sys.stdout.write(CURSOR_UP)
#     sys.stdout.write(ERASE_LINE)
#    pub3.publish(position)
    pub1.publish(angle)
#     pub2.publish(status)    

# If supplied path to video file
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
sleep(2.0)

while True:
    # To reference a specific key press
    key = cv2.waitKey(1) & 0xFF   

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
#     roi_height = height - TOP_H
#     roi_upper_width = int(roi_lower_width - 2*(roi_height/math.tan(alpha_tilt)))
#     print("ROI upper width: {}".format(roi_upper_width))
    # Define Region of Interest (ROI) verticies for perspective transform [x,y]
    roi_bl = [0, height]
    roi_br = [width, height]
    roi_tl = [width//2 - TOP_W//2, TOP_H]
    roi_tr = [width//2 + TOP_W//2, TOP_H]
        
    # Call the frame processing function
    # Play around with hsv values to detect the green lanes in different orientations infront of the camera
    edged = frame_process(resized, color_floor = (80, 60, 60), color_ceiling = (100, 255, 255))

 # Initialize arrays for storing potential points
    warp_y_inds = []
    warp_x_inds = []
#     warp_y_inds = np.zeros(1, (HORIZONTAL_WINDOWS * VERTICAL_WINDOWS))
#     warp_x_inds = np.zeros(1, (HORIZONTAL_WINDOWS * VERTICAL_WINDOWS))

       
    # Perform perspective transform   
    warped = perspective_warp(edged, WARP_W, WARP_H)
    warp_ransaced = perspective_warp(resized, WARP_W, WARP_H)
    # Create a blank-canvas copy of the warped canny-edged image, and the warped original image
    warped_potential = np.zeros_like(warped)
    warp_lanes = np.zeros_like(warp_ransaced)
    
    # Employ sliding window algorithm on warped image (birds-eye view)
    for (x,y, window) in sliding_window(warped, stepX = WARP_WinW, stepY = WARP_WinH, windowSize=(WARP_WinW, WARP_WinH)):
         # If the window is not the proper size ignore it
        if window.shape[0] != WARP_WinH or window.shape[1] != WARP_WinW:
            continue
        # Isolate portion of image within window
        crop = warped[y:y + WARP_WinH, x:x + WARP_WinW]
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
        
    # Loop through the average location points and plot them on the image
    for i in range(0, len(warp_x_inds), 1):
        cv2.circle(warped_potential, (warp_x_inds[i], warp_y_inds[i]), 2, white, -1)
    # Convert the list of x-indicies and y-indicies into an ordered pair list
    avg_points = list(zip(warp_x_inds, warp_y_inds))
        
    # If enough points of interest were found    
    if len(warp_x_inds) > 3:
        # Call function for calculating the first lane (invokes RANSAC)
        avg_remaining, lane_1 = compute_lane(avg_points, 25, 250, 0.3)
        # Call function for drawing first lane
        warp_lanes, topx_1, topy_1 = draw_lane(warp_lanes, lane_1, red, blue)
        # Call function for calculating the second lane (invokes RANSAC)
        leftovers, lane_2 = compute_lane(avg_remaining, 25, 250, 0.3)              
        # Call function for drawing second lane
        warp_lanes, topx_2, topy_2 = draw_lane(warp_lanes, lane_2, red, blue)
        
        # Compute average coordinates of upper lane boundaries
        avg_topx = (topx_1 + topx_2) // 2
        avg_topy = (topy_1 + topy_2) // 2
        
        # Use average lane heading to compute the steering angle and convert to degrees
        steering_angle = math.atan((mid_ref - avg_topx)/(height - avg_topy))
        steering_angle = int(steering_angle * (180/math.pi))
        # Publish steering angle to ROS topic
        ros_talker(steering_angle)
        # Draw the heading on the lanes image
        cv2.line(warp_lanes, (avg_topx, avg_topy), (mid_ref, height), green, 5)        
        # Call inverse warping function
        unwarped = inv_perspective_warp(warp_lanes, width, height)                 
        # Apply the image of drawn lanes onto original image "resized"
        resized = cv2.addWeighted(resized, 1, unwarped, 1, 0)
        # Clean print by writing over previous message
        print("Steering angle: {} degrees".format(steering_angle))
        sys.stdout.write(CURSOR_UP)
        sys.stdout.write(ERASE_LINE)

    # Otherwise, not enough sample points were found (less than 4 in total)
    else:
        # Clean print by writing over previous message
         print("Insufficient data points")       
         sys.stdout.write(CURSOR_UP)
         sys.stdout.write(ERASE_LINE)
    
    # Convert the ROI boundary points to tuple data types for line-drawing 
    vertexA = tuple(roi_bl)
    vertexB = tuple(roi_br)
    vertexC = tuple(roi_tr)
    vertexD = tuple(roi_tl)
    # Draw outline of ROI in bright green on original image for viewer reference
    cv2.line(resized, vertexA, vertexB, green, 2)
    cv2.line(resized, vertexB, vertexC, green, 2)
    cv2.line(resized, vertexC, vertexD, green, 2)
    cv2.line(resized, vertexD, vertexA, green, 2)

    # Display the lane detection results    
    cv2.imshow("Detected Lanes", resized)
#     cv2.imshow("Edges", edged)
#     cv2.imshow("Warped Potential Points", warped_potential)
#     cv2.imshow("Ransaced Points", warp_ransaced)

    # Quit the program if the 'q' key is pressed  
    if key == ord("q"):
        # Kill the ROS serial node and roscore, and wait for successful termination
        os.system("rosnode kill /serial_node")
        roscore.terminate()
        roscore.wait()
        print("Program terminated by user")
        break

# Close all windows and stop the video stream
cv2.destroyAllWindows()
vs.stop()