def gpsToMeters(lat1, lon1, lat2, lon2):
	#Radius of Earth in KM
	R = 6378.137 
	dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000
	
def newGPSfromDist(lat, lon, dist, angle):
	R = 6378137 
	new_lat = 180/math.pi*((dist*math.sin(math.pi*angle/180)/R)+lat*math.pi/180)
	new_lon = 180/math.pi*((dist*math.cos(math.pi*angle/180)/R)+lon*math.pi/180)
	new_coord = [new_lat, new_lon]
	return new_coord
	
def pixelsToGPS(x, y, pix_x, pix_y, bl, tr):
	new_coord = [0, 0]
	dxgps = tr[0] - bl[0]
	dygps = tr[1] - bl[1]
	new_coord[0] = x/pix_x*dxgps + bl[0]
	new_coord[1] = y/pix_y*dygps + bl[1]
	return new_coord

def getSpeedKPH(coord1, coord2, t1, t2):
	dist = gpsToMeters(coord1[0], coord1[1], coord2[0], coord2[1])
	metpersec = dist/(t2 - t1)
	kph = metpersec/3.6
	return kph
	
def getSpeedMPH(coord1, coord2, t1, t2):
	dist = gpsToMeters(coord1[0], coord1[1], coord2[0], coord2[1])
	metpersec = dist/(t2 - t1)
	mph = metpersec/2.23694
	return mph

class Drone:
	def __init__(self, GPS_coord):
		self.speed = 0
		#From 0 to 360 with N=90, E = 0, S = 270, and W = 180
		self.direction = 0
		self.height = 0
		self.lat = GPS_coord[0]
		self.long = GPS_coord[1]
		self.zoom = 1.0
		self.camera_angle = 45
		
# Drone GPS Coordinates
drone_coord = [36.12390612329048, -97.06877946853638]
#Get drone information
drone_inf = Drone(drone_coord)
#Calculate GPS for line of sight
camera_angle = 30
dist_to_center = done_inf.height*math.tan(math.pi*camera_angle/180)
center_coord = newGPSfromDist(drone_inf.lat, drone_inf.lon, dist_to_center, drone_inf.direction)
#Distance from camera to center point of picture
line_of_sight = drone_inf.height*math.cos(math.pi*camera_angle/180)
DFOV = 94
diagonal_length = math.tan(math.pi*DFOV/360)*line_of_sight
x_pix = 1280
y_pix = 720
gamma = math.arctan(y_pix/x_pix)
alpha = diagonal_length/2*math.cos(gamma)
beta = diagonal_length/2*math.sin(gamma)
bl = newGPSfromDist(center_coord[0],center_coord[1], diagonal_length/2, drone_inf.direction + 90 + gamma)
tr = newGPSfromDist(center_coord[0],center_coord[1], diagonal_length/2, drone_inf.direction - 90 + gamma)


