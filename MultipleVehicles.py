#####################################################################

# Integration of Blob Detection and Background Subtraction

# To-Do: Car/shape recognition, detect multiple objects

#####################################################################

import cv2
import argparse
import sys
import math
import numpy as np
from collections import deque

#####################################################################

fullscreen = False; # run in fullscreen mode
use_mog = True; #Use MOG instead of MOG2
input_fps = 30

#Camera's observation angle information, in degrees
cam_diagonal_fov = 94

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
windowNameBGS = "Background Subtraction";

annotation_font = cv2.FONT_HERSHEY_SIMPLEX

measurement = np.array((2,1), np.float32)
prediction = np.zeros((2,1), np.float32)

print("New objects: RED");
print("Tracked Objects: BLUE");
print("Likely cars: GREEN\n");

if use_mog: subtractor = cv2.bgsegm.createBackgroundSubtractorMOG();
else: subtractor = cv2.createBackgroundSubtractorMOG2(history = 20, varThreshold = 10, detectShadows = False)
#Setting blob detection parameters
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByArea = True
params.minArea = 300
params.maxArea = 50000
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
cv2.namedWindow(windowNameBGS, cv2.WINDOW_NORMAL);

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

#Take two rectangles defined by x,y,w,h and get the percentage of rect1 which overlaps rect2
def getOverlaps(rect1, rect2):
	r1_x, r1_y, r1_w, r1_h = rect1
	r2_x, r2_y, r2_w, r2_h = rect2
	return_list = [0, 0]
	if (r1_x < r2_x and r1_x + r1_w < r2_x) or (r2_x < r1_x and r2_x + r2_w < r1_x):
		#No X overlap
		return return_list;
	if (r1_y < r2_y and r1_y + r1_h < r2_y) or (r2_y < r1_y and r2_y + r2_h < r1_y):
		#No Y overlap
		return return_list;
	
	overlap_w = min(r1_x+r1_w, r2_x+r2_w) - max(r1_x, r2_x)
	overlap_h = min(r1_y+r1_h, r2_y+r2_h) - max(r1_y, r2_y)
		
	#Return (area_of_overlap/area_of_rect1, area_of_overlap/area_of_rect2)
	
	return_list = [(overlap_h*overlap_w) / (r1_w*r1_h), (overlap_h*overlap_w) / (r2_w*r2_h)]
	#print(return_list)
	return  return_list

def getMaxOverlap(rect1, rect2):
	overlaps = getOverlaps(rect1, rect2)
	return max(overlaps[0], overlaps[1])


class TrackedObj:
	def __init__(self, start_location):
		self.speed = 0
		self.direction = 0
		self.age = 0
		self.frames_without_detection = 0;
		self.likely_car = False;
		#Stored as x, y, width, height
		self.location = (int(start_location[0]), int(start_location[1]), int(start_location[2]), int(start_location[3]))
		self.foreground_overlaps = []
		self.frames_without_blob_overlap = 0;
		self.past_centers = deque()
		
		
	def updateSpatialHistory(self):
		self.age += 1;
		
		
		
		cntr_x = self.location[0] + self.location[2]/2
		cntr_y = self.location[1] + self.location[3]/2
		cntr = (x,y)
		self.past_centers.append(cntr);
		
		#We use 5 points ago vs this point to get avg speed/direction.
		if self.age < 5:
			return
		
		last_cntr = self.past_centers.popleft()
		
		
		line = (cntr[0] - last_cntr[0], cntr[1] - last_cntr[1])
		
		#prediction = self.kalman.predict()
		#pred_x = prediction[0]
		#pred_y = prediction[1]
		
		cur_speed = math.sqrt( line[0]**2 + line[1]**2 );
		self.speed = cur_speed
		#self.speed = 0.5*cur_speed + 0.5*self.speed
		
		
		#atan2 gets angle in radians from -pi to pi
		cur_direction = math.atan2(line[1], line[0]);
		self.direction = cur_direction

		#self.direction = 0.25*cur_direction + 0.75*self.direction;

# List of current tracked objects
tracked_objs = []

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
	
	#Turn the mask to a color version for drawing rectangles later.
	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR);
	
	
	
	#################################################
	# Figure out which blobs need a new Kalman filter
	
	# Newly detected objects
	# Store as x, y, width, height
	new_detections = []
	
	for point in keypoints:
		centerpoint = point.pt
		
		#Increase diameter b/c blobs always seem smaller than actual object.
		diameter = point.size*1.3;
		radius = diameter/2;
		pt_1x = centerpoint[0] - radius
		pt_2x = centerpoint[0] + radius
		pt_1y = centerpoint[1] - radius
		pt_2y = centerpoint[1] + radius
		
		#Draw all detected blobs on BGS window
		cv2.rectangle(mask, (int(pt_1x), int(pt_1y)), (int(pt_2x), int(pt_2y)), (0,0,255), 3)
		
		# See if this detection is within a currently-tracked vehicle.
		new = True
		for obj in tracked_objs:
			x,y,w,h = obj.location
			if (pt_1x < x+w) and (pt_1y < y+h) and (pt_2x > x) and (pt_2y > y):
				obj.foreground_overlaps.append((pt_1x, pt_1y, diameter, diameter));
				new = False
		# If it's not actually new, continue to check the next points.
		if not new: 
			continue
		
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
	
	#Display BG/blob detection results
	cv2.imshow(windowNameBGS,mask);
		
	############################################################
	# Now that we've (potentially) found new points to track, we
	# need to do initial setup for tracking these new points.
	for to_track in new_detections:
		obj = TrackedObj(to_track)
		
		# init kalman filter object
		obj.kalman = cv2.KalmanFilter(4,2);
		obj.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
		obj.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
		obj.kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03
		
		# convert region to HSV
		pt_1x = int(max(0, obj.location[0]))
		pt_1y = int(max(0, obj.location[1]))
		pt_2x = int(min(frame_width, obj.location[0] + obj.location[2]))
		pt_2y = int(min(frame_height, obj.location[1] + obj.location[3]))
		crop = frame[pt_1y:pt_2y,pt_1x:pt_2x]
		hsv_crop =  cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
		
		# select all Hue (0-> 180) and Sat. values but eliminate values with very low
		# saturation or value (due to lack of useful colour information)
		mask2 = cv2.inRange(hsv_crop, np.array((0., float(s_lower),float(v_lower))), np.array((180.,float(s_upper),float(v_upper))));
		
		# construct a histogram of hue and saturation values and normalize it
		obj.crop_hist = cv2.calcHist([hsv_crop],[0, 1],mask2,[180, 255],[0,180, 0, 255]);
		cv2.normalize(obj.crop_hist,obj.crop_hist,0,255,cv2.NORM_MINMAX);
		obj.crop_hist = obj.crop_hist.reshape(-1)
		
		tracked_objs.append(obj)
		
		#Draw the new detection on the frame in red
		cv2.rectangle(frame,(pt_1x, pt_1y),(pt_2x, pt_2y),(0,0,255),3)
		cv2.imshow(windowNameSelection,frame[pt_1y:pt_2y, pt_1x:pt_2x]);
	
	##########################################
	# Now we start the actual tracking portion
	
	# convert incoming image to HSV
	img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV);
	
	for obj in tracked_objs:
		# First, lets take feedback from previous location, blob detection, and kalman filtering to 
		# guess the next location
		
		#Start with where we found it last time
		expected_loc = list(obj.location);
		
		#Now move it, so the center is the avg of previous and center of our prediction for this time.
		#Only do this if the object has been alive for a couple of frames.
		#This is proving to give a lot of error! Does anyone have a good explanation?
		# if obj.age > 8:
			# prediction = obj.kalman.predict()
			# expected_loc[0] = (prediction[0] + expected_loc[0]+expected_loc[2]/2)/2 - expected_loc[2]/2;
			# expected_loc[1] = (prediction[1] + expected_loc[1]+expected_loc[3]/2)/2 - expected_loc[3]/2;
			# print("After Kalman", expected_loc)
		
		
		#Now use the overlapping blobs, and find the blob closest in size to what we're tracking
		size_differences = []
		for blob in obj.foreground_overlaps:
			size_diff = abs((expected_loc[2]-blob[2])/expected_loc[2]) + abs((expected_loc[3]-blob[3])/expected_loc[3])
			size_differences.append((size_diff, blob));
		
		#sort the list by the size diff value, lowest difference first.
		size_differences = sorted(size_differences, key = lambda x:x[0]);
		
		#Use the one closest in desired size, but only if the overlap is reasonable
		#IE, don't accidentally use a similarly-sized neighboring car.
		#Take chosen blob to home-in on expected location. Use the center of the blob,
		#and the avg of the current width and the blob's width (to allow for slow growth/shrinking,
		#but still slightly avoid tracking two cars w/ one tracker)
		found_overlapping_blob = False
		for size_diff, blob in size_differences:
			if getMaxOverlap(blob, expected_loc) > 0.6:
				found_overlapping_blob = True
				expected_loc[2] = (expected_loc[2] + blob[2])*0.5
				expected_loc[3] = (expected_loc[3] + blob[3])*0.5
				expected_loc[0] = (blob[0]+0.5*blob[2]) - 0.5*expected_loc[2]
				expected_loc[1] = (blob[1]+0.5*blob[3]) - 0.5*expected_loc[3]
				break
		
		if not found_overlapping_blob:
			obj.frames_without_blob_overlap += 1
		else:
			obj.frames_without_blob_overlap = 0
		
		obj.foreground_overlaps = []
		
		expected_loc = (int(expected_loc[0]), int(expected_loc[1]), int(expected_loc[2]), int(expected_loc[3]))
		
	
		# back projection of histogram based on Hue and Saturation only
		img_bproject = cv2.calcBackProject([img_hsv],[0,1],obj.crop_hist,[0,180,0,255],1);
		cv2.imshow(windowName2,img_bproject);

		# apply camshift to predict new location (observation)
		ret, obj.location = cv2.CamShift(img_bproject, expected_loc, term_crit);
		
		x,y,w,h = obj.location;
		#If we found the object, we can update some of the tracking info
		if not ( (x <= 0) or (y <= 0) or ((x+w) >= frame_width) or ((y+h) >= frame_height)):
			#CamShifting keeps exploding the bounding box, so lets force w/h to be expected vals.
			cntr_x = x+0.5*w
			cntr_y = y+0.5*h
			w = expected_loc[2]
			h = expected_loc[3]
			obj.location = (max(0, int(cntr_x-0.5*w)), max(0, int(cntr_y-0.5*h)), w, h);
			x,y,w,h = obj.location;
		
			# Now that we have a new location, lets re-calculate the hue
			crop = frame[y:y+h,x:x+w]
			hsv_crop =  cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
			mask2 = cv2.inRange(hsv_crop, np.array((0., float(s_lower),float(v_lower))), np.array((180.,float(s_upper),float(v_upper))));
			obj.crop_hist = cv2.calcHist([hsv_crop],[0, 1],mask2,[180, 255],[0,180, 0, 255]);
			cv2.normalize(obj.crop_hist,obj.crop_hist,0,255,cv2.NORM_MINMAX);
			obj.crop_hist = obj.crop_hist.reshape(-1)

		# draw observation on image - in BLUE
		frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2);


		# use observation to correct kalman filter
		obj.kalman.correct( np.array([np.float32(x+w/2), np.float32(y+h/2)], np.float32));
		
		
	
	
	
	
	######################################################################
	# Figure out which of these objects can be ignored as not-a-car,
	# for a variety of reasons including:
	#	Object not detected for several sequential frames
	#	Object too large/small based on distance from drone
	#	Object infinitely far from drone (IE flying/above horizon)
	#	Object travelling too slow (Should we do this?)
	#	Object not travelling in the same direction as main traffic body.
	
	
	# Currently very simplistic, doesn't do all of the above.
	for i in range(len(tracked_objs) - 1, -1, -1):
		# Delete nondetections first
		obj = tracked_objs[i]
		
		x,y,w,h = obj.location;
		if ((x <= 0) and ((x+w) >= frame_width)) or ((y <= 0) and ((y+h) >= frame_height)):
			print("Killed for nondetection")
			del tracked_objs[i];
			continue;
		
		# Delete things too big (but not non-detections)
		# Need to refactor to account for distance etc.
		if False and ((w >= frame_width/4) or (h >= frame_height/4)):
			print("Killed for size")
			del tracked_objs[i];
			continue;

		#Delete things tracking something with no movement
		if(obj.frames_without_blob_overlap > 2):
			print("Killed for no blobs")
			del tracked_objs[i]
			continue
		
		#Delete double tracking
		is_duplicate = False;
		for j in range(len(tracked_objs)):
			if j == i: continue;
			
			obj2 = tracked_objs[j]
			overlaps = getOverlaps(obj.location, obj2.location)
			if overlaps[0] > 0.35 and overlaps[1] > 0.35 and obj.age <= obj2.age:
				is_duplicate = True;
				break;
		if is_duplicate:
			print("Killed duplicate")
			del tracked_objs[i]
			continue
		
		#Update movement info
		obj.updateSpatialHistory()
		
		#Tests on movement require that the object have been alive for a bit to be reasonable.
		if obj.age < 8:
			continue
			
		# Delete non-moving
		# Currently considers 2 pixels per frame to be moving.
		# Should be updated to account for distance from drone and fps
		if False and obj.speed < 2 :
			print("Killed for speed")
			del tracked_objs[i]
			continue;
		
		#Draw surviving objects in green.
		frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2);
		frame = cv2.putText(frame, "Car", (x, y), annotation_font, 0.4, (0, 255, 0), 1);

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



