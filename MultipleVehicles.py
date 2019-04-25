import cv2 #opencv, need opencv-contrib-python
import argparse
import sys
import math
from math import radians, degrees, atan, tan, sin, cos, sqrt
import numpy as np  #numpy
import sys
from collections import deque
import multiprocessing
from multiprocessing import Process, Value, Lock
import serial.tools.list_ports #pyserial
from serial import Serial #pyserial


import time

#####################################################################
#Basic configuration options
#####################################################################
use_mog = True; #Use MOG instead of MOG2
print_kill_cause = False
simulate_drone = True
show_bgs = False
show_vid = False

input_fps = 23.98

#Camera's observation angle information, in degrees
cam_diagonal_fov = 94

#Font used for writing annotations on the display.
annotation_font = cv2.FONT_HERSHEY_SIMPLEX

frame_width = 720
frame_height = 404
frame_dims = (frame_height, frame_width, 3)

#####################################################################
#Helper functions
#####################################################################

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

def getAvgOverlap(rect1, rect2):
	overlaps = getOverlaps(rect1, rect2)
	return (overlaps[0] + overlaps[1])/2

#This function returns a rectangle with the same center as reference,
#and with w/l as a weighted average of to_scale and reference, weighted
#in favor of to_scale
def centerAndScale(to_scale, reference, weighting_factor):
	#Set w/l, puts trash in x/y
	to_return = [0,0,0,0]
	to_return[2] = weighting_factor*to_scale[2] + (1-weighting_factor)*reference[2]
	to_return[3] = weighting_factor*to_scale[3] + (1-weighting_factor)*reference[3]
	to_return[0] = reference[0]+0.5*reference[2] - 0.5*to_scale[2]
	to_return[1] = reference[1]+0.5*reference[3] - 0.5*to_scale[3]
	return to_return
	
#Makes some assumptions about 
def pixelsToXY(drone, pix_col, pix_row, num_x_pixels=720, num_y_pixels=404):
	#First, converting everything to rads for Python
	heading = math.radians(drone.direction.value)
	camera_ang = math.radians(drone.camera_angle.value)
	height = drone.height.value
	#This is entirely specific to the camera we are using, the Zenmuse X3
	#diagonal field of view of 94deg and sensor 4:3 aspect ratio, but (vertically) cropped to 16:9 for video
	#FOVs calculated using diagonal fov = 94.5
	h_fov = radians(81.752) # 81.752, 84.0
	v_fov = radians(50.2) # 51.9177, 61.9
		
	#not all pixels are created equal, they do not have the same angle offsets.
	#Calculate a 'unitless focal length' wrt num_y_pixels and v_fov
	f = (num_y_pixels/2)/tan(v_fov/2) #value w/ y and v_fov should be ~same as x and h_fov


	#First going to find y distance from drone, since it is conceptually more clear:
	alpha = atan((0.5*num_y_pixels - pix_row)/f)
	tot_y_angle = camera_ang + alpha
	y_dist = height * tan(tot_y_angle)
		
	#Now to to find x distance:
	beta = atan((pix_col - 0.5*num_x_pixels)/f) #Same assumptions as vertical calculation
	x_dist = tan(beta) * y_dist
	north_dist = -1 * math.sin(heading) * x_dist + math.cos(heading) * y_dist
	east_dist = math.cos(heading) * x_dist + math.sin(heading) * y_dist

	return (north_dist, east_dist)
	
def XYToGPS(north_dist, east_dist, drone):
	#Now to convert to GPS angles
	r_earth = 6378137 
	#(m) This will be the radius to define our radians
	dr_lat = radians(drone.lat.value)
	dr_long = radians(drone.long.value)
	#Latitude is fortunately linear:
	north_radians = north_dist / r_earth
	obj_lat = north_radians + dr_lat
	#Longitude is a bit tricker, because we need equivalent earth radius:
	equiv_r_earth = r_earth * cos(obj_lat)
	east_radians = east_dist / equiv_r_earth
	obj_long = east_radians + dr_long
		
	#Convert back to degrees and output
	obj_lat = degrees(obj_lat)
	obj_long = degrees(obj_long)
	lat_long_obj = [obj_lat, obj_long]
	return lat_long_obj


#Finds the distance in meters between two gps coordinates
def gpsToMeters(lat1, lon1, lat2, lon2):
	#Radius of Earth in KM
	R = 6378.137 
	dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
	dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
	a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = R * c
	return d * 1000

#Takes two coordinates at two times to figure out the speed of the object in kilometers per hour
def getSpeedKPH(coord1, coord2, t1, t2):
	dist = gpsToMeters(coord1[0], coord1[1], coord2[0], coord2[1])
	metpersec = dist/(t2 - t1)
	kph = metpersec/3.6
	return kph

#Takes two coordinates at two times to figure out the speed of the object in miles per hour	
def getSpeedMPH(coord1, coord2, time):
	dist = gpsToMeters(coord1[0], coord1[1], coord2[0], coord2[1])
	metpersec = dist/time
	mph = metpersec/2.23694
	return mph
	
def getCoordClick(event, x, y, flags, param):
	drone = param
	if event == cv2.EVENT_LBUTTONDOWN:
		xy_coords = pixelsToXY(drone, x, y, num_x_pixels=720, num_y_pixels=404)
		gps_coords = XYToGPS(xy_coords[0], xy_coords[1], drone)
		print(str(x)+ ", "+ str(y) + " = " +str(xy_coords[0])+", "+str(xy_coords[1])+"="+ str(gps_coords[0])+ "," + str(gps_coords[1]))
	
	
#Converts string to float and stores in val.value
#Increments miss on failure, miss=0 on success
def str_to_float_helper(val, string, miss):
	try:
		val.value = float(string)
		num_misses = 0
	except ValueError:
		num_misses += 1


##################################################################################
#Class representing a single object being tracked
##################################################################################
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
		self.gps = [0,0]
		self.in_queue = False
		self.end_of_queue = False
		self.time_in_queue = 0
		
		
	def updateSpatialHistory(self, drone):
		self.age += 1;
		if self.in_queue:
			self.time_in_queue+=1
		else:
			self.time_in_queue = 0
		
		cntr_x = self.location[0] + self.location[2]/2
		cntr_y = self.location[1] + self.location[3]/2
		phys_cntr = pixelsToXY(drone, cntr_x, cntr_y)
		self.past_centers.append(phys_cntr);
		self.gps = XYToGPS(phys_cntr[0], phys_cntr[1], drone)
		#We use 5 points ago vs this point to get avg speed/direction.
		if self.age < 5:
			return
		
		last_cntr = self.past_centers.popleft()
		
		line = (phys_cntr[0] - last_cntr[0], phys_cntr[1] - last_cntr[1])
		past_gps = XYToGPS(last_cntr[0], last_cntr[1], drone)
		#prediction = self.kalman.predict()
		#pred_x = prediction[0]
		#pred_y = prediction[1]
		time = 5*1/input_fps #seconds since last point
		
		#cur_speed = math.sqrt( line[0]**2 + line[1]**2 ) / time
		#self.speed = cur_speed * 0.0223694 #cm/s to miles/hour
		if self.age%10 == 0:
			self.speed = 5*getSpeedMPH(self.gps, past_gps, time)
		
		#atan2 gets angle in radians from -pi to pi
		cur_direction = math.atan2(line[1], line[0]);
		self.direction = cur_direction

		#self.direction = 0.25*cur_direction + 0.75*self.direction;


##################################################################################
#Class representing the state of the drone, for 
##################################################################################
class Drone:
	def __init__(self, drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long):
		self.speed = drone_speed
		self.direction = drone_direction
		self.height = drone_height
		self.lat = drone_lat
		self.long = drone_long
		self.zoom = 1.0
		self.camera_angle = drone_cam_direction



def tracking_setup(blur_size, min_blob_size, detection_buffer):
	#Set up our background detection padding amount. (should be odd)
	conv_size = detection_buffer.value
	#conv_kernel = np.ones((conv_size,conv_size), dtype=np.float32)
	conv_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(conv_size,conv_size))
	
	#Set up the background subtractor.
	if use_mog: subtractor = cv2.bgsegm.createBackgroundSubtractorMOG();
	else: subtractor = cv2.createBackgroundSubtractorMOG2(history = 20, varThreshold = 10, detectShadows = False)
	
	#Setting blob detection parameters
	params = cv2.SimpleBlobDetector_Params()
	params.filterByColor = True
	params.blobColor = 255
	params.filterByArea = True
	params.minArea = min_blob_size.value
	params.maxArea = 500000000
	params.filterByCircularity = False
	params.filterByConvexity = False
	params.filterByInertia = False
	# Set up the detector with set parameters.
	detector = cv2.SimpleBlobDetector_create(params)
	
	# Setup the termination criteria for hist backprojection, either 10 iteration or
	# move by at most 1 pixel pos. difference
	term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 1 )
	
	
	return (conv_kernel, subtractor, detector, term_crit)

def do_track(cap_src, name, blur_size, min_blob_size, detection_buffer, change_made, drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long, output_frame):
	
	cap = cv2.VideoCapture();
	if not cap.open(cap_src):
		print("Unable to open the capture source", cap_src, "for", name);
		return;
		
	
	output_frame_np = np.frombuffer(output_frame.get_obj(), dtype=np.uint8)
	output_frame_np.shape = frame_dims
	
	
	#Set the framerate of whatever input we use
	#Doesn't seem to work? Commented out for now, will need to investigate.
	#cap.set(cv2.CAP_PROP_FPS, 10)
	
	#Drone object to keep drone state
	drone_state = Drone(drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long)

	# define display window names
	windowName = name + ": Vehicle Tracking"; # window name
	windowNameBGS = name + ": Background Subtraction";
	
	setup_objs = tracking_setup(blur_size, min_blob_size, detection_buffer)
	conv_kernel = setup_objs[0]
	subtractor = setup_objs[1]
	detector = setup_objs[2]
	term_crit = setup_objs[3]
	
	#Variable used to control frame display
	frame_by_frame = False;
	
	
	#Kalman filter measurement/prediction vals (used to force type)
	measurement = np.array((2,1), np.float32)
	prediction = np.zeros((2,1), np.float32)
	
	
	# List of current tracked objects
	tracked_objs = []

	# We'll grab a frame before we begin processing, just to pull
	# info about frame size. Skipping one frame should be NBD.
	ret, frame = cap.read();
	# gets the image size (channel is not used)
	frame_height, frame_width, channel = frame.shape
	
	
	################################################################
	#Tracking loop
	################################################################
	while (True):
	
		if change_made.value:
			with change_made.get_lock():
				setup_objs = tracking_setup(blur_size, min_blob_size, detection_buffer)
				conv_kernel = setup_objs[0]
				subtractor = setup_objs[1]
				detector = setup_objs[2]
				term_crit = setup_objs[3]
				
				change_made.value = False

		# if video file successfully open then read frame from video
		if (cap.isOpened):
			ret, frame = cap.read();
		else:
			print("Video unable to open")
			return;
		if ret == False:
			print("Unable to read any more");
			break; #Assume for now that we shouldn't keep trying?

		# start a timer (to see how long processing and display takes)
		start_t = cv2.getTickCount();
		
		
		# Applying background subtraction
		blur_size_val = blur_size.value;
		blur = cv2.GaussianBlur(frame, (blur_size_val,blur_size_val), 0)
		mask = subtractor.apply(blur)
		
		# Add a white pixel around each existing white pixel,
		# to help bridge tiny gaps in BGS
		# First, we have to make a black border around the edge, to provide a default edge-case of 0
		mask = cv2.copyMakeBorder(mask, 2,2,2,2, cv2.BORDER_CONSTANT, value=0)
		mask = cv2.filter2D(mask, -1, conv_kernel);
		# Now remove the extra border pixels
		mask = mask[2:-2, 2:-2];
		
		# Detect blobs.
		keypoints = detector.detect(mask)
		
		#Turn the mask to a color version for drawing rectangles later.
		if show_bgs: mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR);
		
		
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
			if show_bgs: cv2.rectangle(mask_color, (int(pt_1x), int(pt_1y)), (int(pt_2x), int(pt_2y)), (0,0,255), 3)
			
			# See if this detection is within a currently-tracked vehicle.
			new = True
			for obj in tracked_objs:
				x,y,w,h = obj.location
				if (pt_1x < x+w) and (pt_1y < y+h) and (pt_2x > x) and (pt_2y > y):
					obj.foreground_overlaps.append((pt_1x, pt_1y, diameter, diameter));
					if getAvgOverlap(obj.location, (pt_1x,pt_1y,diameter,diameter)) > 0.5:
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
		if show_bgs: cv2.imshow(windowNameBGS,mask_color);
			
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
			
			tracked_objs.append(obj)
			
		
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
				if getAvgOverlap(blob, expected_loc) > 0.6:
					found_overlapping_blob = True
					expected_loc = centerAndScale(expected_loc, blob, 0.9)
					break
			
			if not found_overlapping_blob:
				obj.frames_without_blob_overlap += 1
			else:
				obj.frames_without_blob_overlap = 0
			
			obj.foreground_overlaps = []
			
			expected_loc = (int(expected_loc[0]), int(expected_loc[1]), int(expected_loc[2]), int(expected_loc[3]))
			

			# apply camshift to predict new location (observation)
			ret, obj.location = cv2.meanShift(mask, expected_loc, term_crit);
			
			x,y,w,h = obj.location;
			#If we found the object, we can update some of the tracking info
			if not ( ((x <= 0) and ((x+w) >= frame_width)) or ((y <= 0) and ((y+h) >= frame_height))):
				#CamShifting keeps exploding the bounding box, so lets force w/h to be expected vals.
				# obj.location = centerAndScale(expected_loc, obj.location, 0.9)
				# obj.location = (int(obj.location[0]), int(obj.location[1]), int(obj.location[2]), int(obj.location[3]))
				# x,y,w,h = obj.location;
				pass

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
		
		has_queue = False
		queue = []
		# Currently very simplistic, doesn't do all of the above.
		for i in range(len(tracked_objs) - 1, -1, -1):
			# Delete nondetections first
			obj = tracked_objs[i]
			
			x,y,w,h = obj.location;
			if ((x <= 0) and ((x+w) >= frame_width)) or ((y <= 0) and ((y+h) >= frame_height)):
				if print_kill_cause: print("Killed for nondetection")
				del tracked_objs[i];
				continue;
			
			# Delete things too big (but not non-detections)
			# Need to refactor to account for distance etc.
			if False and ((w >= frame_width/4) or (h >= frame_height/4)):
				if print_kill_cause: print("Killed for size")
				del tracked_objs[i];
				continue;

			#Delete things tracking something with no movement
			if(obj.frames_without_blob_overlap > max(min(0.5*(obj.age-obj.frames_without_blob_overlap), 10), 3)):
				if print_kill_cause: print("Killed for no blobs")
				del tracked_objs[i]
				continue
			
			#Delete double tracking
			if obj.age < 8:
				is_duplicate = False;
				for j in range(len(tracked_objs)):
					if j == i: continue;
					
					obj2 = tracked_objs[j]
					overlaps = getOverlaps(obj.location, obj2.location)
					if overlaps[0] > 0.35 and overlaps[1] > 0.35 and obj.age <= obj2.age:
						is_duplicate = True;
						break;
				if is_duplicate:
					if print_kill_cause: print("Killed duplicate")
					del tracked_objs[i]
					continue
			
			#Update movement info
			obj.updateSpatialHistory(drone_state)
			
			# Delete non-moving
			# Currently considers 2 pixels per frame to be moving.
			# Should be updated to account for distance from drone and fps
			if False and obj.speed < 2 :
				if print_kill_cause: print("Killed for speed")
				del tracked_objs[i]
				continue;
			#Find end of queue
			if obj.speed < 25 and ((obj.direction < math.pi and obj.direction > 7*math.pi/8) or (obj.direction > -math.pi and obj.direction < -7*math.pi/8)):
				if len(queue) == 0:
					has_queue = True
					obj.end_of_queue = True
				else:
					for car in queue:
						if car.end_of_queue and car.location[0] < obj.location[0]:
							car.end_of_queue = False
							obj.end_of_queue = True	
				queue.append(obj)
				obj.in_queue = True
			elif obj.in_queue:
				obj.in_queue = False
				obj.end_of_queue = False

		for obj in tracked_objs:
			#Tests on movement require that the object have been alive for a bit to be reasonable.
			if obj.age < 8:
				is_duplicate = False;
				
			if obj.end_of_queue and obj.time_in_queue>5:
				frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255),2)
				frame = cv2.putText(frame, '%.1f MPH' % obj.speed, (x, y-3), annotation_font, 0.4, (0, 0, 255), 1)
				frame = cv2.putText(frame, 'lat: %.6f' % obj.gps[0], (x, y-23), annotation_font, 0.4, (0, 0, 255), 1)
				frame = cv2.putText(frame, 'long: %.6f' % obj.gps[1], (x, y-13), annotation_font, 0.4, (0, 0, 255), 1)
			else:
				#Draw surviving objects in green.
				frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2);
				frame = cv2.putText(frame, '%.1f MPH' % obj.speed, (x, y-3), annotation_font, 0.4, (0, 255, 0), 1);

		# display image
		if show_vid: 
			cv2.imshow(windowName,frame);
			cv2.setMouseCallback(windowName, getCoordClick, drone_state)
		#Write image to shared memory:
		output_frame_np[...] = frame[...]
		
		# stop the timer and convert to ms. (to see how long processing and display takes)
		stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000;
		#print(stop_t);

		################################################################################################
		# start the event loop - essential

		# wait 40ms or less depending on processing time taken (i.e. 1000ms / 25 fps = 40 ms)
		key = cv2.waitKey(max(2, 40 - int(math.ceil(stop_t)))) & 0xFF;
		
		if (key == ord('f')):
			frame_by_frame = not frame_by_frame
		
		while frame_by_frame:
			key = cv2.waitKey(2) & 0xFF;
			if (key == ord('x')):
				break;
			if (key == ord('f')):
				frame_by_frame = not frame_by_frame
		
		if not frame_by_frame:
			if (key == ord('x')):
					break;






def drone_comm(serial_to_use, drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long):
	ser = Serial(serial_to_use, 9600, timeout=0.3)
	
	code_to_obj = {
		"dsp":drone_speed,
		"dht":drone_height,
		"ddr":drone_direction,
		"cdr":drone_cam_direction,
		"dla":drone_lat,
		"dlo":drone_long
	}
	
	num_misses = 0
	while True:
		line = ser.readline()
		if len(line) < 5:
			num_misses += 1
			continue
		if line[0:2] in code_to_obj:
			str_to_float_helper(code_to_obj[line[0:2]], line[4:], num_misses)
		else:
			num_misses += 1
			
			
			
###############################################################################################
#GUI
###############################################################################################
from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1234, 721)
        self.horizontalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 1232, 721))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self.horizontalLayout_3.setSpacing(1)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.src_input = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.src_input.sizePolicy().hasHeightForWidth())
        self.src_input.setSizePolicy(sizePolicy)
        self.src_input.setMinimumSize(QtCore.QSize(400, 20))
        self.src_input.setObjectName("src_input")
        self.horizontalLayout_3.addWidget(self.src_input)
        self.src_button = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.src_button.sizePolicy().hasHeightForWidth())
        self.src_button.setSizePolicy(sizePolicy)
        self.src_button.setObjectName("src_button")
        self.horizontalLayout_3.addWidget(self.src_button)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem = QtWidgets.QSpacerItem(20, 10, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_2 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_5.addWidget(self.label_2)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.horizontalLayout_5.addWidget(self.lineEdit_2)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.radioButton_2 = QtWidgets.QRadioButton(self.horizontalLayoutWidget)
        self.radioButton_2.setObjectName("radioButton_2")
        self.horizontalLayout_4.addWidget(self.radioButton_2)
        self.radioButton = QtWidgets.QRadioButton(self.horizontalLayoutWidget)
        self.radioButton.setObjectName("radioButton")
        self.horizontalLayout_4.addWidget(self.radioButton)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_18 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_18.setObjectName("horizontalLayout_18")
        self.label_10 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_18.addWidget(self.label_10)
        self.queue_speed_input = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.queue_speed_input.setObjectName("queue_speed_input")
        self.horizontalLayout_18.addWidget(self.queue_speed_input)
        self.pushButton = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_18.addWidget(self.pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_18)
        self.line = QtWidgets.QFrame(self.horizontalLayoutWidget)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem1)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.blur_slider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.blur_slider.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blur_slider.sizePolicy().hasHeightForWidth())
        self.blur_slider.setSizePolicy(sizePolicy)
        self.blur_slider.setMinimumSize(QtCore.QSize(20, 0))
        self.blur_slider.setMaximumSize(QtCore.QSize(20, 16777215))
        self.blur_slider.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.blur_slider.setAcceptDrops(False)
        self.blur_slider.setToolTip("")
        self.blur_slider.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.blur_slider.setAutoFillBackground(False)
        self.blur_slider.setMinimum(3)
        self.blur_slider.setMaximum(31)
        self.blur_slider.setSingleStep(2)
        self.blur_slider.setProperty("value", 11)
        self.blur_slider.setOrientation(QtCore.Qt.Vertical)
        self.blur_slider.setInvertedAppearance(False)
        self.blur_slider.setInvertedControls(False)
        self.blur_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.blur_slider.setTickInterval(2)
        self.blur_slider.setObjectName("blur_slider")
        self.horizontalLayout_7.addWidget(self.blur_slider)
        self.verticalLayout_4.addLayout(self.horizontalLayout_7)
        spacerItem2 = QtWidgets.QSpacerItem(101, 0, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem2)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_3 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setMinimumSize(QtCore.QSize(0, 0))
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_12.addWidget(self.label_3)
        self.verticalLayout_4.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_6.addLayout(self.verticalLayout_4)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem3)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setSpacing(0)
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.blob_slider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.blob_slider.setMinimum(30)
        self.blob_slider.setMaximum(500)
        self.blob_slider.setPageStep(50)
        self.blob_slider.setOrientation(QtCore.Qt.Vertical)
        self.blob_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.blob_slider.setTickInterval(50)
        self.blob_slider.setObjectName("blob_slider")
        self.horizontalLayout_9.addWidget(self.blob_slider)
        self.verticalLayout_5.addLayout(self.horizontalLayout_9)
        self.label_4 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_4.setMinimumSize(QtCore.QSize(101, 0))
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_5.addWidget(self.label_4)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem4)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.buffer_slider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.buffer_slider.setMinimum(1)
        self.buffer_slider.setMaximum(9)
        self.buffer_slider.setSingleStep(2)
        self.buffer_slider.setPageStep(2)
        self.buffer_slider.setProperty("value", 5)
        self.buffer_slider.setOrientation(QtCore.Qt.Vertical)
        self.buffer_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.buffer_slider.setTickInterval(2)
        self.buffer_slider.setObjectName("buffer_slider")
        self.horizontalLayout_10.addWidget(self.buffer_slider)
        self.verticalLayout_6.addLayout(self.horizontalLayout_10)
        self.label_5 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_6.addWidget(self.label_5)
        self.horizontalLayout_6.addLayout(self.verticalLayout_6)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        spacerItem6 = QtWidgets.QSpacerItem(20, 5, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem6)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.graphicsView = QtWidgets.QLabel(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.graphicsView.sizePolicy().hasHeightForWidth())
        self.graphicsView.setSizePolicy(sizePolicy)
        self.graphicsView.setMinimumSize(QtCore.QSize(720, 405))
        self.graphicsView.setBaseSize(QtCore.QSize(720, 405))
        self.graphicsView.setObjectName("graphicsView")
        self.verticalLayout_2.addWidget(self.graphicsView)
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_6 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_14.addWidget(self.label_6)
        self.EOQ_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.EOQ_Label.sizePolicy().hasHeightForWidth())
        self.EOQ_Label.setSizePolicy(sizePolicy)
        self.EOQ_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.EOQ_Label.setObjectName("EOQ_Label")
        self.horizontalLayout_14.addWidget(self.EOQ_Label)
        self.verticalLayout_7.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_9 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_15.addWidget(self.label_9)
        self.roadsign_gps_input = QtWidgets.QLineEdit(self.horizontalLayoutWidget)
        self.roadsign_gps_input.setAlignment(QtCore.Qt.AlignCenter)
        self.roadsign_gps_input.setObjectName("roadsign_gps_input")
        self.horizontalLayout_15.addWidget(self.roadsign_gps_input)
        self.verticalLayout_7.addLayout(self.horizontalLayout_15)
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_7 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_16.addWidget(self.label_7)
        self.label_8 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_16.addWidget(self.label_8)
        self.verticalLayout_7.addLayout(self.horizontalLayout_16)
        self.horizontalLayout_13.addLayout(self.verticalLayout_7)
        self.graphicsView_2 = QtWidgets.QGraphicsView(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.graphicsView_2.sizePolicy().hasHeightForWidth())
        self.graphicsView_2.setSizePolicy(sizePolicy)
        self.graphicsView_2.setMinimumSize(QtCore.QSize(275, 275))
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.horizontalLayout_13.addWidget(self.graphicsView_2)
        self.verticalLayout_2.addLayout(self.horizontalLayout_13)
        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Work Zone UAV Project"))
        self.label.setText(_translate("Dialog", "Source"))
        self.src_button.setText(_translate("Dialog", "Ok"))
        self.label_2.setText(_translate("Dialog", "Road Sign IP:"))
        self.radioButton_2.setText(_translate("Dialog", "Normal Speed (65 MPH)"))
        self.radioButton.setText(_translate("Dialog", "Slow down (35MPH)"))
        self.label_10.setText(_translate("Dialog", "Queue Speed:"))
        self.pushButton.setText(_translate("Dialog", "PushButton"))
        self.label_3.setText(_translate("Dialog", "Blur Radius"))
        self.label_4.setText(_translate("Dialog", "Minimum Blob Size"))
        self.label_5.setText(_translate("Dialog", "Detection Buffer Size"))
        self.label_6.setText(_translate("Dialog", "End of Queue (deg N,E):"))
        self.EOQ_Label.setText(_translate("Dialog", "N/A"))
        self.label_9.setText(_translate("Dialog", "Road Sign position (deg N,E):"))
        self.label_7.setText(_translate("Dialog", "Distance:"))
        self.label_8.setText(_translate("Dialog", "TextLabel"))

class Thread(QtCore.QThread):
	changePixmap = QtCore.pyqtSignal(QtGui.QImage)

	def __init__(self, *args, **kwargs):
		QtCore.QThread.__init__(self, *args, **kwargs)
		self.flag = False

	def run(self):
		cap1 = cv2.VideoCapture(0)
		self.flag = True
		frame = np.frombuffer(self.output_frame.get_obj(), dtype=np.uint8)
		frame.shape = frame_dims
		while self.flag:
			time.sleep(1/input_fps)
			rgb_image = np.flip(frame, 2).copy()
			cvt2qt = QtGui.QImage(rgb_image, frame_dims[1], frame_dims[0], QtGui.QImage.Format_RGB888)
			self.changePixmap.emit(cvt2qt)                         # I don't really understand this yet

	def stop(self):
		self.flag = False


class Prog(QtWidgets.QMainWindow, Ui_Dialog):
	def __init__(self, output_frame, blur_size, min_blob_size, detection_buffer, change_made, src_dict, src_set):
		super().__init__()
		self.setupUi(self)
		
		self.blur_size = blur_size
		self.min_blob_size = min_blob_size
		self.detection_buffer = detection_buffer
		self.change_made = change_made
		self.src_dict = src_dict
		self.src_set = src_set
		
		self.th = Thread(self)
		self.th.output_frame = output_frame
		self.th.changePixmap.connect(self.setImage)
		self.th.start()
		
		self.blob_slider.sliderReleased.connect(self.update_blobs)
		self.blur_slider.sliderReleased.connect(self.update_blur)
		self.buffer_slider.sliderReleased.connect(self.update_buffer)
		self.src_button.clicked.connect(self.update_src)

	@QtCore.pyqtSlot(QtGui.QImage) 
	def setImage(self, image):
		self.graphicsView.setPixmap(QtGui.QPixmap.fromImage(image))

	def closeEvent(self, event):
		self.th.stop()
		self.th.wait()
		super().closeEvent(event)
	
	def update_blobs(self):
		with self.change_made.get_lock():
			self.min_blob_size.value = int(self.blob_slider.value())
			self.change_made.value = True
	
	def update_blur(self):
		with self.change_made.get_lock():
			to_use = int(self.blur_slider.value())
			if to_use % 2 == 0:
				to_use += 1
			self.blur_size.value = to_use
			self.change_made.value = True
	
	def update_buffer(self):
		with self.change_made.get_lock():
			to_use = int(self.buffer_slider.value())
			if to_use % 2 == 0:
				to_use += 1
			self.detection_buffer.value = to_use
			self.change_made.value = True
			
	def update_src(self):
		with self.src_set.get_lock():
			self.src_set.value = True
			self.src_dict["src"] = self.src_input.text()
			
		

def open_gui(output_frame, blur_size, min_blob_size, detection_buffer, change_made, src_dict, src_set):
	Program =  QtWidgets.QApplication(sys.argv)
	MyProg = Prog(output_frame, blur_size, min_blob_size, detection_buffer, change_made, src_dict, src_set)
	MyProg.show()
	Program.exec_()
	
	
	

if __name__ == '__main__':

	print("Tracked Objects: BLUE");
	print("Likely cars: GREEN\n");

	###########################################################################################
	#Find Correct serial for drone communication (IE find correct USB port
	###########################################################################################
	serial_process = None
	
	#All of these are lockless, they are write only from one process and read only from the rest, 
	#no locks needed.
	drone_speed = Value('d', 0.0, lock=False)
	drone_height = Value('d', 2940, lock=False) 
	drone_direction = Value('d', 90.0, lock=False) 
	drone_cam_direction = Value('d', -10.0, lock=False)
	drone_lat = Value('d', 36.12390612329048, lock=False)
	drone_long = Value('d', -97.06877946853638, lock=False)
	
	if not simulate_drone:
		print("Searching for drone communication link...", end ="");
		ports = list(serial.tools.list_ports.comports())
		found = False
		for p in ports:
			ser = Serial(port=p[0],baudrate=9600,timeout=0.5)
			for i in range(3):
				line = ser.readline().decode("utf-8")
				if re.match('[a-z,A-Z][a-z,A-Z][a-z,A-Z]:*', line) != None:
					print(" found!")
					found = True
					serial_to_use = p[0]
					ser.close()
					break
				print(".", end ="")
			ser.close()
			if found:
				break
			
		if not found:
			print(" unable to find port, aborting!")
			exit()
		serial_process = Process(target=drone_comm, args=(serial_to_use, drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long) );
		serial_process.start()
	
	
	
	
	############################################################################################
	#Setup video capture
	############################################################################################
	# parse command line arguments for camera ID or video file
	parser = argparse.ArgumentParser(description='Perform ' + sys.argv[0] + ' example operation on incoming camera/video image')
	parser.add_argument("-c", "--camera_to_use", type=int, help="specify camera to use", default=0)
	parser.add_argument('video_file', metavar='video_file', type=str, nargs='?', help='specify optional video file')
	args = parser.parse_args()
		
	if args.video_file:
		cap_src = str(args.video_file);
	else:
		cap_src = args.camera_to_use;
		
	
	###########################################################################################
	#Configure mutable parameters
	###########################################################################################
	blur_size = Value('i', 11, lock=False);
	min_blob_size = Value('i', 30, lock=False);
	detection_buffer = Value('i', 5, lock=False);
	
	#Tells the tracking to re-init any changed values.
	change_made = Value('i', True);
	src_set = Value('i', False);
	manager = multiprocessing.Manager()
	src_dict = manager.dict()
	
	#Output from tracking, array representing the frame.
	output_frame = multiprocessing.Array('B', int(np.prod(frame_dims)));
	
	gui_process = Process(target=open_gui, args=(output_frame, blur_size, min_blob_size, detection_buffer, change_made, src_dict, src_set))
	gui_process.start()
	
	tracking_process = None
	while gui_process.is_alive():
		if src_set.value == True:
			with src_set.get_lock():
				if not tracking_process == None:
					tracking_process.terminate();
				print("Running w/ src", src_dict["src"])
				tracking_process = Process(target=do_track, args=(src_dict["src"], "Feed 1", blur_size, min_blob_size, detection_buffer, change_made, drone_speed, drone_height, drone_direction, drone_cam_direction, drone_lat, drone_long, output_frame));
				tracking_process.start();
				src_set.value = False
		
	
	
	
	#Do cleanup once the gui is closed.
	if not tracking_process == None:
		tracking_process.terminate();
	if not simulate_drone:
		serial_process.terminate()
	
	cv2.destroyAllWindows()