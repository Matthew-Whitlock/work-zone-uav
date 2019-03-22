#####################################################################

# Integration of Blob Detection and Background Subtraction

# To-Do: Car/shape recognition, detect multiple objects

#####################################################################

import cv2
import argparse
import sys
import math
import numpy as np

#####################################################################

keep_processing = True;
selection_in_progress = False; # support interactive region selection
fullscreen = False; # run in fullscreen mode

# parse command line arguments for camera ID or video file

parser = argparse.ArgumentParser(description='Perform ' + sys.argv[0] + ' example operation on incoming camera/video image')
parser.add_argument("-c", "--camera_to_use", type=int, help="specify camera to use", default=0)
parser.add_argument('video_file', metavar='video_file', type=str, nargs='?', help='specify optional video file')
args = parser.parse_args()

#####################################################################

# return centre of a set of points representing a rectangle

def center(points):
	x = np.float64((points[0][0] + points[1][0] + points[2][0] + points[3][0]) / 4.0)
	y = np.float64((points[0][1] + points[1][1] + points[2][1] + points[3][1]) / 4.0)
	return np.array([np.float32(x), np.float32(y)], np.float32)

#####################################################################

# this function is called as a call-back everytime the trackbar is moved
# (here we just do nothing)

def nothing(x):
	pass

#####################################################################
# define video capture object

cap = cv2.VideoCapture();

# define display window name

windowName = "Vehicle Tracking"; # window name
windowName2 = "Hue histogram back projection"; # window name
windowNameSelection = "initial selected region";


measurement = np.array((2,1), np.float32)
prediction = np.zeros((2,1), np.float32)

print("\nBlob Detection: RED");
print("Observation in image: BLUE");
print("Prediction from Kalman: GREEN\n");

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

if (((args.video_file) and (cap.open(str(args.video_file))))
	or (cap.open(args.camera_to_use))):
    
	# Create Windows
	cv2.namedWindow(windowName, cv2.WINDOW_NORMAL);
	cv2.namedWindow(windowName2, cv2.WINDOW_NORMAL);
	cv2.namedWindow(windowNameSelection, cv2.WINDOW_NORMAL);

	# set sliders for HSV selection thresholds

	s_lower = 60;
	cv2.createTrackbar("s lower", windowName2, s_lower, 255, nothing);
	s_upper = 255;
	cv2.createTrackbar("s upper", windowName2, s_upper, 255, nothing);
	v_lower = 32;
	cv2.createTrackbar("v lower", windowName2, v_lower, 255, nothing);
	v_upper = 255;
	cv2.createTrackbar("v upper", windowName2, v_upper, 255, nothing);

	cropped = False

	# Setup the termination criteria for search, either 10 iteration or
	# move by at least 1 pixel pos. difference
	term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
	
	#kalman filter lists
	kalmans = []
	crop_hists = []
	track_windows = []
    
	while (keep_processing):

		# if video file successfully open then read frame from video

		if (cap.isOpened):
			ret, frame = cap.read();

		# start a timer (to see how long processing and display takes)

		start_t = cv2.getTickCount();

        # get parameters from track bars

		s_lower = cv2.getTrackbarPos("s lower", windowName2);
		s_upper = cv2.getTrackbarPos("s upper", windowName2);
		v_lower = cv2.getTrackbarPos("v lower", windowName2);
		v_upper = cv2.getTrackbarPos("v upper", windowName2);
        
		if ret == True:
            #Applying background subtraction
			mask = subtractor.apply(frame)
			# Detect blobs.
			keypoints = detector.detect(mask)
			for point in keypoints:     
				centerpoint = point.pt
				new = True
				for track_window in track_windows:
					x,y,w,h = track_window;
					if((centerpoint[0] >= x) and (centerpoint[0] <= (x+w)) and (centerpoint[1] >= y) and (centerpoint[1] <= (y+h))):
						new = False
				if new == True:
					point1x = int(centerpoint[0] - (point.size/2))
					point1y = int(centerpoint[1] - (point.size/2))
					point2x = int(centerpoint[0] + (point.size/2))
					point2y = int(centerpoint[1] + (point.size/2))
					cv2.rectangle(frame,(point1x, point1y),(point2x, point2y),(0,0,255),3)
					crop = frame[point1y:point2y,point1x:point2x].copy()
					h, w, c = crop.shape;   # size of template
					if (h > 0) and (w > 0):
						cropped = True;
						# init kalman filter object

						kalman = cv2.KalmanFilter(4,2)
						kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)

						kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)

						kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03
						kalmans.append(kalman)
                        # convert region to HSV

						hsv_crop =  cv2.cvtColor(crop, cv2.COLOR_BGR2HSV);
                    
                        # select all Hue (0-> 180) and Sat. values but eliminate values with very low
                        # saturation or value (due to lack of useful colour information)

						mask2 = cv2.inRange(hsv_crop, np.array((0., float(s_lower),float(v_lower))), np.array((180.,float(s_upper),float(v_upper))));
                        # mask = cv2.inRange(hsv_crop, np.array((0., 60.,32.)), np.array((180.,255.,255.)));

                        # construct a histogram of hue and saturation values and normalize it

						crop_hist = cv2.calcHist([hsv_crop],[0, 1],mask2,[180, 255],[0,180, 0, 255]);
						cv2.normalize(crop_hist,crop_hist,0,255,cv2.NORM_MINMAX);
						crop_hists.append(crop_hist)

                        # set intial position of object

						track_window = (point1x,point1y,point2x - point1x,point2y - point1y);
						track_windows.append(track_window)

						cv2.imshow(windowNameSelection,crop);
        # if we have a selected region

		if (cropped):
            #gets the image size (channel is not used)
			frame_width, frame_height, channel = frame.shape
			
            # convert incoming image to HSV

			img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);
			
			delete_filters = []
			
			for vehicle in range(len(track_windows)):

			    # back projection of histogram based on Hue and Saturation only

				img_bproject = cv2.calcBackProject([img_hsv],[0,1],crop_hists[vehicle],[0,180,0,255],1);
				cv2.imshow(windowName2,img_bproject);

                # apply camshift to predict new location (observation)
				ret, track_windows[vehicle] = cv2.CamShift(img_bproject, track_windows[vehicle], term_crit);

				# draw observation on image - in BLUE
				x,y,w,h = track_windows[vehicle];
				if( (x <= 0) or (y <= 0) or ((x+w) >= frame_width) or ((y+h) >= frame_height)):
					delete_filters.append(vehicle)
				frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2);

                # extract centre of this observation as points

				pts = cv2.boxPoints(ret)
				pts = np.int0(pts)
                # (cx, cy), radius = cv2.minEnclosingCircle(pts)

                # use to correct kalman filter

				kalmans[vehicle].correct(center(pts));

                # get new kalman filter prediction

				prediction = kalmans[vehicle].predict();

                # draw predicton on image - in GREEN

				frame = cv2.rectangle(frame, (prediction[0]-(0.5*w),prediction[1]-(0.5*h)), (prediction[0]+(0.5*w),prediction[1]+(0.5*h)), (0,255,0),2);
			for i in range(len(delete_filters) - 1, -1, -1):
				filter = delete_filters[i]
				del crop_hists[filter]
				del track_windows[filter]
				del kalmans[filter]
			if len(kalmans) == 0:
				cropped = False

		else:

            # before we have cropped anything show the mask we are using
            # for the S and V components of the HSV image

			img_hsv =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);

            # select all Hue values (0-> 180) but eliminate values with very low
            # saturation or value (due to lack of useful colour information)

			mask2 = cv2.inRange(img_hsv, np.array((0., float(s_lower),float(v_lower))), np.array((180.,float(s_upper),float(v_upper))));

			cv2.imshow(windowName2,mask2);
        # display image

		cv2.imshow(windowName,frame);
		cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN & fullscreen);
                
        # stop the timer and convert to ms. (to see how long processing and display takes)

		stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000;

        # start the event loop - essential

        # cv2.waitKey() is a keyboard binding function (argument is the time in milliseconds).
        # It waits for specified milliseconds for any keyboard event.
        # If you press any key in that time, the program continues.
        # If 0 is passed, it waits indefinitely for a key stroke.
        # (bitwise and with 0xFF to extract least significant byte of multi-byte response)
        # here we use a wait time in ms. that takes account of processing time already used in the loop

        # wait 40ms or less depending on processing time taken (i.e. 1000ms / 25 fps = 40 ms)

		key = cv2.waitKey(max(2, 40 - int(math.ceil(stop_t)))) & 0xFF;

        # It can also be set to detect specific key strokes by recording which key is pressed

        # e.g. if user presses "x" then exit  / press "f" for fullscreen display

		if (key == ord('x')):
			keep_processing = False;
		elif (key == ord('f')):
			fullscreen = not(fullscreen);
        # close all windows

	cv2.destroyAllWindows()

else:
	print("No video file specified or camera connected.");



