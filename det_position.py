#Knowns are: Lat, Long, Height, camera_angle, x&y pixel location, Heading (Compass Direction)
from math import radians, degrees, atan, tan, sin, cos, sqrt

def det_position(dr_lat, dr_long, height, camera_ang_deg, heading, pix_col, pix_row, num_x_pixels=1920, num_y_pixels=1080):
    #First, converting everything to rads for Python
    dr_lat = radians(dr_lat)
    dr_long = radians(dr_long)
    heading = radians(heading)    
    camera_ang = radians(camera_ang_deg)
    #assume diagonal field of view of 94deg and 16:9 aspect ratio
    v_fov = radians(61.9)#From wikipedia "Angle of View" for 20mm equivalent 2019-04-11
    h_fov = radians(82.4)#Also from wiki
    camera_ang = radians(camera_ang_deg)
    
    #First going to find y distance from drone, since it is conceptually more clear:
    dif_alpha = v_fov / num_y_pixels #Assume that each row of pixels are at the same alpha where dif_alpha is the change in alpha between each row
    alpha = (pix_row - 539.5) * dif_alpha #Assume that bottom row is labeled as 0, and top as 1079
    tot_y_angle = camera_ang + alpha
    y_dist = height * tan(tot_y_angle)
    print(y_dist)
    #Now to to find x distance:
    dif_beta = h_fov / num_x_pixels
    beta = (pix_col - 959.5) *dif_beta #Same assumptions as vertical calculation
    tot_dist = sqrt(height*height + y_dist*y_dist) #Just a guess for the moment
    x_dist = height * tan(beta)
    print(x_dist)
    north_dist = -1 * sin(heading) * x_dist + cos(heading) * y_dist
    east_dist = cos(heading) * x_dist + sin(heading) * y_dist
    
    #Okay, great! Now to convert to GPS angles
    r_earth = 6371800 #(m) This will be the radius to define our radians
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
    
#Testing cases
dr_lat = 36.11561   #Drone Latitude (In this case, it's the lat/long of Stillwater)
dr_long = -97.05837 #Drone Longitude
height = 100        #height of drone in meters
camera_ang_deg = 0 #Angle of camera from drone in degrees(0 is straight down)
heading = 0         #Due north is 0 degrees, with positive direction defined as clockwise. (e.g 90 is due east)
pix_col = 1919       #Whichever column the pixel is on (i.e. x-coordinate) (0 based indexing assumed)
pix_row = 1079       #Whichever row the pixel is on (i.e. y-coordinate)

lat_long = det_position(dr_lat, dr_long, height, camera_ang_deg, heading, pix_col, pix_row)
print("Lat: " + str(lat_long[0]))
print("Long: " + str(lat_long[1]))
