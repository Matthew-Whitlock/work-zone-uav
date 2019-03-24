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

fullscreen = False; # run in fullscreen mode
input_fps = 30

# parse command line arguments for camera ID or video file

parser = argparse.ArgumentParser(description='Perform ' + sys.argv[0] + ' example operation on incoming camera/video image')
parser.add_argument("-c", "--camera_to_use", type=int, help="specify camera to use", default=0)
parser.add_argument('video_file', metavar='video_file', type=str, nargs='?', help='specify optional video file')
args = parser.parse_args()

#####################################################################

# this function is called as a call-back everytime the trackbar is moved
# (here we just do nothing)

def nothing(x):
	pass

#####################################################################
# define video capture object

cap = cv2.VideoCapture();

#Set the framerate of whatever input we use
#Doesn't seem to work? Commented out for now, will need to investigate.
#cap.set(cv2.CAP_PROP_FPS, 10)

# define display window name

windowName = "Vehicle Tracking"; # window name
windowName2 = "Hue histogram back projection"; # window name
windowNameSelection = "initial selected region";


measurement = np.array((2,1), np.float32)
prediction = np.zeros((2,1), np.float32)

print("New objects: RED");
print("Tracked Objects: BLUE");
print("Likely cars: GREEN\n");

subtractor = cv2.createBackgroundSubtractorMOG2(history = 2*input_fps, varThreshold = 15, detectShadows = False)
#Setting blob detection parameters
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByArea = True
params.minArea = 300
params.maxArea = 10000
params.filterByCircularity = False
params.filterByConvexity = False
#params.filterByInertia = False

# Set up the detector with set parameters.
detector = cv2.SimpleBlobDetector_create(params)

# Open video, check that we are successful
if not (((args.video_file) and (cap.open(str(args.video_file))))
	or (cap.open(args.camera_to_use))):
	
	print("No video file specified or camera connected.");
	exit();

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

# Setup the termination criteria for search, either 10 iteration or
# move by at least 1 pixel pos. difference
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

# kalman filter lists
kalmans = [] # Active Kalman Filters
crop_hists = [] # Hue

# Currently detected 'cars' from tracking
# Store as x, y, width, height
obj_locations = []

# Details used for distinguishing cars from noise.
# Initial detected location, stored as (x,y) for center of object.
start_locations = []
obj_ages = []


# We'll grab a frame before we begin processing, just to pull
# info about frame size. Skipping one frame should be NBD.
if (cap.isOpened):
	ret, frame = cap.read();
	# gets the image size (channel is not used)
	frame_height, frame_width, channel = frame.shape

while (True):

	# if video file successfully open then read frame from video
	if (cap.isOpened):
		ret, frame = cap.read();
	if ret == False:
		print("Unable to read any more");
		break; #Assume for now that we shouldn't keep trying?

	# start a timer (to see how long processing and display takes)
	start_t = cv2.getTickCount();

	# get parameters from track bars
	s_lower = cv2.getTrackbarPos("s lower", windowName2);
	s_upper = cv2.getTrackbarPos("s upper", windowName2);
	v_lower = cv2.getTrackbarPos("v lower", windowName2);
	v_upper = cv2.getTrackbarPos("v upper", windowName2);
	
	
	# Applying background subtraction
	mask = subtractor.apply(frame)
	
	# Detect blobs.
	keypoints = detector.detect(mask)
	
	
	
	#################################################
	# Figure out which blobs need a new Kalman filter
	
	# Newly detected objects
	# Store as x, y, width, height
	new_detections = []
	
	for point in keypoints:
		centerpoint = point.pt			
		
		# See if this detection is within a currently-tracked vehicle.
		new = True
		for track_window in obj_locations:
			x,y,w,h = track_window;
			if (centerpoint[0] >= x) and (centerpoint[0] <= (x+w)) and (centerpoint[1] >= y) and (centerpoint[1] <= (y+h)):
				new = False
				break;
		# If it's not actually new, continue to check the next points.
		if not new: 
			continue
		
		diameter = point.size;
		radius = diameter/2;
		pt_1x = centerpoint[0] - radius
		pt_2x = centerpoint[0] + radius
		pt_1y = centerpoint[1] - radius
		pt_2y = centerpoint[1] + radius
		
		# See if this detection overlaps another new detection
		# If so, just choose the largest detection.
		for to_check in new_detections:
			x,y,w,h = to_check
			if (pt_1x < x+w) and (pt_1y < y+h) and (pt_2x > x) and (pt_2y > y):
				#We have an overlapping point
				new = False;
				#If this new point is bigger, replace the old.
				#Since to_check is a brand new detection, we know width=height=diameter.
				if(diameter > w):
					# Replace values.
					to_check[0] = pt_1x;
					to_check[1] = pt_1y;
					to_check[2] = diameter;
					to_check[3] = diameter;
					
				break;
		if new:
			new_detections.append([pt_1x, pt_1y, diameter, diameter]);
		
		
		
	############################################################
	# Now that we've (potentially) found new points to track, we
	# need to do initial setup for tracking these new points.
	for to_track in new_detections:
		point1x = max(0, int(to_track[0]))
		point1y = max(0, int(to_track[1]))
		point2x = min(frame_width, int(to_track[0] + to_track[2]))
		point2y = min(frame_height, int(to_track[1] + to_track[3]))
		
		start_locations.append((to_track[0]+to_track[2]/2, to_track[1]+to_track[3]/2))
		obj_ages.append(0);
		
		#Draw the new detection on the frame in red
		cv2.rectangle(frame,(point1x, point1y),(point2x, point2y),(0,0,255),3)
			
		# init kalman filter object
		kalman = cv2.KalmanFilter(4,2)
		kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)

		kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)

		kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03
		kalmans.append(kalman)
		
		# convert region to HSV
		crop = frame[point1y:point2y,point1x:point2x].copy()
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
		obj_locations.append((int(to_track[0]), int(to_track[1]), int(to_track[2]), int(to_track[3])))

		cv2.imshow(windowNameSelection,crop);
	
	
	
	
	##########################################
	# Now we start the actual tracking portion
	
	# convert incoming image to HSV
	img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);
	
	for vehicle in range(len(obj_locations)):
		# back projection of histogram based on Hue and Saturation only
		img_bproject = cv2.calcBackProject([img_hsv],[0,1],crop_hists[vehicle],[0,180,0,255],1);
		cv2.imshow(windowName2,img_bproject);

		# apply camshift to predict new location (observation)
		ret, obj_locations[vehicle] = cv2.CamShift(img_bproject, obj_locations[vehicle], term_crit);

		# draw observation on image - in BLUE
		x,y,w,h = obj_locations[vehicle];
		frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2);

		# use observation to correct kalman filter
		kalmans[vehicle].correct( np.array([x+w/2, y+h/2], np.float32));
	
	
	
	
	######################################################################
	# Figure out which of these objects can be ignored as not-a-car,
	# for a variety of reasons including:
	#	Object not detected for several sequential frames
	#	Object too large/small based on distance from drone
	#	Object infinitely far from drone (IE flying/above horizon)
	#	Object travelling too slow (Should we do this?)
	#	Object not travelling in the same direction as main traffic body.
	
	# Currently very simplistic, doesn't do all of the above.
	for i in range(len(obj_locations) - 1, -1, -1):
		obj_ages[i] = obj_ages[i] + 1
		# Delete nondetections first
		x,y,w,h = obj_locations[i];
		if( (x <= 0) or (y <= 0) or ((x+w) >= frame_width) or ((y+h) >= frame_height)):
			del crop_hists[i]
			del obj_locations[i]
			del kalmans[i]
			del start_locations[i]
			del obj_ages[i]
			continue;
		
		# Delete things too big
		# Need to refactor to account for distance etc.
		if (w >= frame_width/4) or (h >= frame_height/4):
			del crop_hists[i]
			del obj_locations[i]
			del kalmans[i]
			del start_locations[i]
			del obj_ages[i]
			continue;
		
		#Some tests require that the object have been alive for a bit to be reasonable.
		if obj_ages[i] < 10:
			continue
			
		# Delete non-moving
		# Currently considers 2 pixels per frame to be moving.
		# Should be updated to account for distance from drone and fps
		cntr_x = x+(w/2)
		cntr_y = y+(h/2)
		if (math.sqrt((start_locations[i][0] - cntr_x)**2 + (start_locations[i][1] - cntr_y)**2)/obj_ages[i]) < 2.5 :
			del crop_hists[i]
			del obj_locations[i]
			del kalmans[i]
			del start_locations[i]
			del obj_ages[i]
			continue;
		
		#Draw surviving objects in green.
		frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2);


	# display image

	cv2.imshow(windowName,frame);
	cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN & fullscreen);
			
	# stop the timer and convert to ms. (to see how long processing and display takes)
	stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000;
	#print(stop_t);

	################################################################################################
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
		break;
	elif (key == ord('f')):
		fullscreen = not(fullscreen);


# close all windows
cv2.destroyAllWindows()



