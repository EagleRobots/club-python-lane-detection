# python-lane-detection-ransac
Robust real-time lane detection algorithm incorporating RANSAC and perspective transform

This code is developed to be run in an environment with OpenCV and relevant libraries (numpy, imutils, pip, etc) already installed. Either a webcam or Picamera can be used to provide real-time video stream.

The file titled "ransac_trajectory.py" contains the most recent execution code as well as most of the functions used. The file titled "ransac_function_linear" contains the full linear RANSAC algoritm used to perform robust real-time lane detection. Additionally, "lane_following.ino" contains the Arduino code responsible for creating the appropriate ROS subscriber for the Arduino to handle and process incoming steering angles from the "ransac_trajectory" script.
