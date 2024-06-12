#!/usr/bin/env python3

import serial
import socket
import rospy
from numpy import arctan2
from sensor_msgs.msg import NavSatFix

class GPS:
    def __init__(self) -> None:
        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout = 1)
        self.pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size = 10)

    def convert(self, value):
        value = float(value)
        degrees = int(value / 100)
        minutes = (value % 100)
        return (degrees + minutes / 60)

    def gps_pub(self):
        self.msg = NavSatFix()
        self.msg.header.frame_id = "gps"
        msg_raw = str(self.ser.readline())
        raw_arr = msg_raw.split(",")
        if raw_arr[0] == "b'$GNGGA" and raw_arr[2] != '':            
            self.msg.status.status = int(raw_arr[6])
            self.msg.latitude = self.convert(raw_arr[2])
            self.msg.longitude = self.convert(raw_arr[4])
            self.msg.altitude = float(raw_arr[9])        
        elif raw_arr[2] == '':
            rospy.loginfo("engine kaput, lost signal")

        self.pub.publish(self.msg)

while not rospy.is_shutdown():
    try:
        rospy.init_node('gps_pub', anonymous= True)
        gps = GPS()
        gps.gps_pub()
    except KeyboardInterrupt:
        print("Shutting down")
        break
