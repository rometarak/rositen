#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import math
import csv
import pymap3d as pm
from std_msgs.msg import Float32
from scipy.optimize import minimize

class Robot:
    def __init__(self):
        rospy.Subscriber('/gps/fix', NavSatFix, self.callback)

        # Publisher for the wheels
        self.left_pub = rospy.Publisher('Left_vel', Float32, queue_size=10)
        self.right_pub = rospy.Publisher('Right_vel', Float32, queue_size=10)
        self.w = 0.54
        self.theta = 0
        self.vl = 0.00
        self.vr = 0.00

    def callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude

    def get_enu(self):
        self.enu = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, 58.3428685594, 25.5692475361, 91.357)
        return self.enu

    def send_vel(self, left, right):
        left_msg = Float32()
        left_msg.data = left
        self.left_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = right
        self.right_pub.publish(right_msg)

def calculate_heading(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    bearing = math.atan2(delta_y, delta_x)
    bearing = math.degrees(bearing)
    return bearing

def calculate_error_angle(current_heading, target_bearing):
    error_angle = (target_bearing - current_heading + 360) % 360
    if error_angle > 180:
        error_angle -= 360
    return error_angle

def mpc_controller(robot, x, y, marker, dt):
    def robot_model(state, u, dt):
        x, y, theta = state
        vl, vr = u
        w = robot.w
        x += ((vl + vr) / 2) * math.cos(theta) * dt
        y -= ((vl + vr) / 2) * math.sin(theta) * dt
        theta += (vr - vl) / w * dt
        return x, y, theta

    def cost_fn(u):
        state = [x, y, robot.theta]
        cost = 0
        for k in range(horizon):
            state = robot_model(state, u[k*2:k*2+2], dt)
            cost += (state[0] - marker[0])**2 + (state[1] - marker[1])**2
            cost += 0.00003 * (u[k*2]**2 + u[k*2+1]**2)  # Penalize high velocities
        return cost

    horizon = 10
    u0 = [robot.vl, robot.vr] * horizon

    # Speed constraints for the wheels
    max_wheel_speed = 100  # Set to the appropriate max speed for your robot
    bounds = [(-max_wheel_speed, max_wheel_speed)] * horizon * 2

    res = minimize(cost_fn, u0, bounds=bounds, method='SLSQP')
    if res.success:
        return res.x[:2]
    else:
        rospy.logerr("Optimization failed: %s", res.message)
        return robot.vl, robot.vr

lasttime = 0
i = 0

previous_error = 0.0
dt = 1e-6

points = []
with open('/home/roisten/catkin_ws/recordings/11-06-2024-10-52.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, quotechar='|')
    for row in spamreader:
        x = float(row[0])
        y = float(row[1])
        z = float(row[2])
        points.append([x, y, z])

robot = Robot()
rospy.init_node('Robot', anonymous=True)

while not rospy.is_shutdown():
    try:
        prev = robot.get_enu()
        prevX, prevY, prevZ = prev

        current_time = rospy.get_time()
        dt = (current_time - lasttime)
        lasttime = current_time
        x, y, z = robot.get_enu()

        if x != prevX or y != prevY:
            heading = calculate_heading(prevX, prevY, x, y)
            prevX, prevY = x, y
            marker_heading = calculate_heading(x, y, points[i][0], points[i][1])
            error = calculate_error_angle(heading, marker_heading)
            rospy.loginfo("Error: %s", error)

            vl, vr = mpc_controller(robot, x, y, points[i], dt)
            rospy.loginfo("Calculated velocities: vl=%s, vr=%s", vl, vr)

            # Send velocity commands to the robot
            robot.send_vel(vl, vr)

            # Threshold to move to the next point
            Inx = points[i][0] - x
            Iny = points[i][1] - y
            In = math.sqrt(Inx**2 + Iny**2)
            rospy.loginfo("Distance to next point: %s", In)
            if In <= 4:
                i += 1

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        break

