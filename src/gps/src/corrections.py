#!/usr/bin/env python3

import serial
import socket
import rospy

HOST = '213.168.5.170'
PORT = 8002
ser = serial.Serial('/dev/ttyACM1', 115200, timeout = 1)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    while not rospy.is_shutdown():
        try:
            ser.write(s.recv(1024))
        except KeyboardInterrupt:
            break
