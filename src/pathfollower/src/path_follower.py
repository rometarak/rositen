#!/usr/bin/env python3

import numpy as np
import math
import rospy
import scipy
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
    lin_x = data.linear.x
    lin_y = data.linear.y
    lin_z = data.linear.z

    ang_x = data.angular.x
    ang_y = data.angular.y
    ang_z = data.angular.z
    
    #cmd_vel andmed
    print("linear: ", lin_x,lin_y,lin_z, "angular: ",ang_x,ang_y,ang_z)

#odomeetria andmed saame odometry failist(teeme funktsiooniks selle, kui vaja)

def predict(x0, model, control_signals, step_size):
    time = 0
    dt = step_size # min(step_size, 1.0e-3)
    u = None
    x = x0.copy()
    result = []
    def sample():
    	res = [time]
    	res.extend([v[0] for v in x])
    	res.extend([_u for _u in u])
    	result.append(res)

    for u in control_signals:
    	sample()
    	# for i in range(0, math.ceil(step_size / dt)):
    	k1 = model(x, u)
    	k2 = model(x + k1 * 0.5 * dt, u)
    	k3 = model(x + k2 * 0.5 * dt, u)
    	k4 = model(x + k3 * dt, u)
    	x = x + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6.0
    sample()

    return result

def model(x, u):
    m = 100
    g = 9.80665
    w = 0.3
    I = 5

    return np.array([
		[float(x[2])],
		[float(x[3])],
		[-100.0 * (u[0] + u[1]) * math.sin(float(x[4])) / m],
		[100.0 * (u[0] + u[1]) * math.cos(float(x[4])) / m - g],
		[float(x[5])],
		[100.0 * (u[1] - u[0]) * w / 2.0 / I],
    ])

x0 = np.array([
	[0.0],
	[0.0],
	[0.0],
	[0.0],
	[0.0],
	[0.0],
])

r = np.array([
	[4.0],
	[0.0],
	[0.0],
	[0.0],
	[0.0],
	[0.0],
])

def control(x):
    def cost_fnc(us):
        _us = []
        for i in range(int(len(us) / 2)):
            _us.append([us[i*2], us[i*2 + 1]])
            states = predict(x, model, _us, time_step)

	    # evaluate cost
            cost = 0
            for state in states:
                ex = r[0,0] - state[1]
                ey = r[1,0] - state[2]
                evx = r[2,0] - state[3]
                evy = r[3,0] - state[4]
                eo = r[4,0] - state[5]
                ew = r[5,0] - state[6]
                cost += 5*ex*ex + 5*ey*ey + 1 * eo * eo;
    
        return cost
    us = np.zeros(2 * horizont)
    bounds = []
    for u in us:
        bounds.append((0.0, 10.0))
        us = scipy.optimize.minimize(cost_fnc, us, bounds=bounds).x
        return np.array([us[0], us[1]])

rospy.init_node('signal', anonymous=True)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
rate = rospy.Rate(50)
rospy.spin()

