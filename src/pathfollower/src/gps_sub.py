#!/usr/bin/env python3
import rospy
import csv
from datetime import datetime
from sensor_msgs.msg import NavSatFix
import math

now = datetime.now()
now = now.strftime("%d-%m-%Y-%H-%M")
csvfilename = '/home/roisten/catkin_ws/recordings/' + str(now) + '.csv'
latitude = 0
longitude = 0
altitude = 0

a = 6378137.0                            
b = 6356752.314245
f = (a - b) / a
f_inv = 1.0 / f

a_sq = a * a
b_sq = b * b
e_sq = f * (2 - f)


def gps_callback(msg):
    global latitude
    global longitude
    global altitude
    x,y,z = geotoecef(latitude,longitude,altitude)
    print(csvfilename)
    with open(csvfilename, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(efctoenu(x,y,z, 58.3428685594, 25.5692475361, 91.357))
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude


def geotoecef(lat, lon, h):
    # Convert to radians in notation consistent with the paper:
    lambda0 = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lambda0)
    N = a / math.sqrt(1 - e_sq * s * s)
    sin_lambda = math.sin(lambda0)
    cos_lambda = math.cos(lambda0)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x,y,z

#N - 58.3428685594
#E - 25.5692475361
#H - 91.357

# double x, double y, double z, double lat0, double lon0, double h0, out double xEast, out double yNorth, out double zUp
def efctoenu(x, y, z, lat0, lon0, h0):
    # Convert to radians in notation consistent with the paper:
    lambda0 = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lambda0)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lambda0)
    cos_lambda = math.cos(lambda0)
    cos_phi = math.cos(phi);
    sin_phi = math.sin(phi);

    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    # This is the matrix multiplication
    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
        
    return xEast, yNorth, zUp


rospy.init_node('recordpath', anonymous=True)
rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)
rate = rospy.Rate(10)

rospy.spin()

