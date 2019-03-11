# Standard imports
import cv2
import sys
import numpy as np
import tkinter as tk

root = tk.Tk()
#Get screen dimensions
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
 
#Video from file
cap = cv2.VideoCapture('highway.mp4')
#Video from camera
#cap = cv2.VideoCapture(0)
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
subtractor = cv2.createBackgroundSubtractorMOG2(history = 20, varThreshold = 15, detectShadows = False)
#Setting blob detection parameters
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByArea = True
params.minArea = 300
params.maxArea = 100000
params.filterByCircularity = False
params.filterByConvexity = False
#params.filterByInertia = False
# Set up the detector with set parameters.
detector = cv2.SimpleBlobDetector_create(params)
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()
  if ret == True:
    #Applying background subtraction
    mask = subtractor.apply(frame)
    # Detect blobs.
    keypoints = detector.detect(mask)
    for point in keypoints:
        centerpoint = point.pt
        point1x = int(centerpoint[0] - (point.size/2))
        point1y = int(centerpoint[1] - (point.size/2))
        point2x = int(centerpoint[0] + (point.size/2))
        point2y = int(centerpoint[1] + (point.size/2))
        cv2.rectangle(frame,(point1x, point1y),(point2x, point2y),(0,0,255),3)
        #cv2.rectangle(mask,(point1x, point1y),(point2x, point2y),(0,0,255),3)
    # Show keypoints
    cv2.namedWindow("Background Subtracted", cv2.WINDOW_NORMAL)
    cv2.moveWindow("Background Subtracted",0,0)
    cv2.imshow("Background Subtracted",mask)
    cv2.namedWindow("Keypoints", cv2.WINDOW_NORMAL)
    #cv2.moveWindow("Keypoints",int(screen_width/2),0)
    cv2.imshow("Keypoints", frame)
 
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object and exit
cap.release()
cv2.destroyAllWindows()
