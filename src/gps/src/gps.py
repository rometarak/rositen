#!/usr/bin/env python3

import serial
import socket
import rospy
from sensor_msgs.msg import NavSatFix

HOST = '213.168.5.170'
PORT = 8002
ser = serial.Serial('/dev/ttyACM1', 115200, timeout = 1) 

def convert(value):
    value = float(value)
    degrees = int(value / 100)
    minutes = (value % 100)
    return (degrees + minutes / 60)

def gps_pub(prev_corr_t):
    pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size = 10)
    rospy.init_node('gps_pub', anonymous= True)
    rate = rospy.Rate(10)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT)) 
        msg = NavSatFix()
        msg.header.frame_id = "gps"
        msg_raw = str(ser.readline())
        raw_arr = msg_raw.split(",")

        if raw_arr[0] == "b'$GNGGA" and raw_arr[2] != '':          #TODO : Disable $GNTXT in Ucentre            
            msg.status.status = int(raw_arr[6])
            msg.latitude = convert(raw_arr[2])
            msg.longitude = convert(raw_arr[4])
            msg.altitude = float(raw_arr[9])
            if int(raw_arr[6]) < 4 or rospy.get_time() - prev_corr_t > 1.0:
                ser.write(s.recv(256))                #send corrections
                prev_corr_t = rospy.get_time()
            pub.publish(msg)
        rate.sleep()

    return prev_corr_t

prev_corr_t = 0

while True:
    prev_corr_t = gps_pub(prev_corr_t)
