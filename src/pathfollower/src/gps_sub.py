#!/usr/bin/env python3
import rospy
import csv
from datetime import datetime
from sensor_msgs.msg import NavSatFix
import pymap3d as pm

now = datetime.now()
now = now.strftime("%d-%m-%Y-%H-%M")
csvfilename = '/home/roisten/catkin_ws/recordings/' + str(now) + '.csv'

def gps_callback(msg):
    x = msg.latitude
    y = msg.longitude
    z = msg.altitude
    with open(csvfilename, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(pm.geodetic2enu(x,y,z, 58.3428685594, 25.5692475361, 91.357))
    
rospy.init_node('recordpath', anonymous=True)
rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)
rate = rospy.Rate(10)

rospy.spin()

