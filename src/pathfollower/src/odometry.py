#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import String

ticksRight = 0
ticksLeft = 0

prevTicksLeft = 0.0
prevTicksRight = 0.0

leftRotation = 0
rightRotation = 0

leftDistance = 0
rightDistance = 0

lastlastRight = 0
lastlastLeft = 0

distanceTraveled = 0
degreesTurned = 0
deltaX = 0
deltaY = 0

# Wheel radius
wheelRadius = 0.125
#Distance between 2 wheels
L = 0.52
# Full circle
ticksPerRevolution = 90

lastRight = "000"
lastLeft = "000"

# hall0 = right ticks, hall1 = right reverse, hall2 = left ticks, hall3 = left reverse
def odometry_callback(data):
    global lastRight
    global lastLeft

    global ticksRight
    global ticksLeft
        
    hallSensor = data.data
    hall = hallSensor.split()
    hallRight = hall[0] #hallSensor[:3]

    hallLeft = hall[1]  #hallSensor[3:]
    #hallRightReverse = hallSensor[6:7]
    #hallLeftReverse = hallSensor[7:]

    #print(hallRight,hallLeft,hallRightReverse,hallLeftReverse)

    if lastRight != hallRight: #and hallRightReverse != "1":
        ticksRight = ticksRight + 1
    #elif hallRightReverse != "0":
    #    ticksRight = ticksRight - 1

    if lastLeft != hallLeft: #and hallLeftReverse != "1":
        ticksLeft = ticksLeft + 1
   # elif hallLeftReverse != "0":
   #     ticksLeft = ticksLeft - 1
   
    lastRight = hallRight
    lastLeft = hallLeft
   
    # Calculate rotations of each wheel
    leftRotation = ticksLeft * ((2 * math.pi) / ticksPerRevolution)
    rightRotation = ticksRight * ((2 * math.pi) / ticksPerRevolution)

    # Calculate distance traveled of each wheel
    leftDistance = wheelRadius * leftRotation
    rightDistance = wheelRadius * rightRotation

    # Distance traveled for both wheels combined
    distanceTraveled = (rightDistance+leftDistance) / 2

    # Calculate how much we have turned since the start point
    degreesTurned = (rightDistance-leftDistance) / L

    # Calculate the x and y coordinates from start point
    deltaX = distanceTraveled*math.cos(degreesTurned)
    
    deltaY = distanceTraveled*math.sin(degreesTurned)
    print(rightDistance, leftDistance)  
    print('Distance traveled: ' + str(round(distanceTraveled,3)),'Degrees turned: '+ str(math.degrees(round(degreesTurned,3))))
    print("----------------------")

rospy.init_node('pathfollower', anonymous=True)
rospy.Subscriber('/wheelticks', String, odometry_callback)
rate = rospy.Rate(50)
rospy.spin()
