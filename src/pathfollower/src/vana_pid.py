#!/usr/bin/env python3

import math
import time
import csv
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import pymap3d as pm

class PID:
    def __init__(self):
        self.sub = rospy.Subscriber("/gps/fix", NavSatFix, self.get_gps_coordinates)
        self.longitude = 0.0
        self.latitude = 0.0
        self.altitude = 0.0
    
    # Assume you have a function to get current GPS coordinates of the robot
    def get_gps_coordinates(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.altitude = data.altitude
        
        x, y, z = pm.geodetic2enu(self.latitude, self.longitude, self.altitude, 58.3428685594, 25.5692475361, 91.357)
        
        return x, y, z

    def calculate_heading(self, last_position, current_position):
        # Extract latitude and longitude from the arrays
        lat1 = last_position
        lon1 = last_position
        lat2 = current_position
        lon2 = current_position
    
        # Convert degrees to radians
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
    
        # Compute the difference in longitude
        delta_lon = lon2 - lon1
    
        # Calculate the heading
        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    
        initial_heading = math.atan2(x, y)
    
        # Convert from radians to degrees
        initial_heading = math.degrees(initial_heading)
    
        # Normalize to 0-360
        compass_heading = (initial_heading + 360) % 360
    
        return compass_heading

    #def calculate_distance(self, other_point):
        # Calculate the distance between two points using Haversine formula
        #lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        #lat2, lon2 = math.radians(other_point.latitude), math.radians(other_point.longitude)
        
        #dlat = lat2 - lat1
        #dlon = lon2 - lon1
        #a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        #c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        #distance = 6371 * c  # Earth radius in kilometers

    #return distance

    #def find_nearest_point(self, points):
        #distance = math.sqrt((point[0] - self.latitude)**2 + (point[1] - self.longitude)**2)
        #min_distance = float('inf')
        #nearest_point = None

        #for point in points:
            #distance = math.sqrt((point[0] - self.latitude)**2 + (point[1] - self.longitude)**2)
            #distance = self.calculate_distance(point)
            #if distance < min_distance:
            #    min_distance = distance
            #    nearest_point = point

        #return nearest_point

        # vaja leida error joonest
        # vaja leida error suunast
        

    # Assume you have a function to calculate the distance and angle between two GPS coordinates
    def calculate_distance_and_heading(self, target_gps):
        # Placeholder function, replace with actual implementation
        target_lat, target_lon = target_gps
        
        # Calculate the distance between current and target GPS coordinates
        distance = math.sqrt((target_lat - self.latitude)**2 + (target_lon - self.longitude)**2)
        
        # Calculate the heading angle towards the target
        heading = math.atan2(target_lon - self.longitude, target_lat - self.latitude)
        
        return distance, heading

    def control_robot(self, heading):
        # Placeholder function, replace with actual implementation
        # Convert desired heading to left and right wheel velocities
        
        # Define maximum velocity (adjust as per your robot's capabilities)
        max_velocity = 1.0
        
        # Define wheel separation distance (distance between left and right wheels)
        wheel_base = 0.5  # Example value, adjust based on your robot's dimensions
        
        # Calculate differential velocities for left and right wheels based on the desired heading
        desired_angular_velocity = k_p * heading  # Proportional control (adjust k_p as needed)
        
        # Limit the maximum angular velocity
        desired_angular_velocity = min(max(desired_angular_velocity, -max_velocity), max_velocity)
        
        # Calculate individual wheel velocities
        vel_left = desired_angular_velocity * wheel_base / 2.0
        vel_right = -desired_angular_velocity * wheel_base / 2.0
        
        # Send velocity commands to the robot
        self.send_velocity_commands(vel_left, vel_right)

    # Placeholder function to send velocity commands to the robot
    def send_velocity_commands(self, left, right):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('send_command', anonymous=True)
        rate = rospy.Rate(10)
        pub.publish(left, right)
        rate.sleep()

    # Main function for GPS navigation
    def navigate_to_target(self, target_gps):
        while True:
            current_gps = self.latitude, self.longitude
            distance, heading = self.calculate_distance_and_heading(target_gps)
            
            if distance < tolerance:
                print("Reached target destination")
                break
            
            # Adjust heading to ensure it falls within [-pi, pi] range
            heading = (heading + math.pi) % (2 * math.pi) - math.pi
            
            self.control_robot(heading)
            
            # Adjust this sleep time based on the loop frequency required for your application
            time.sleep(0.1)

# Example usage
if __name__ == "__main__":
    pid_controller = PID()
    # Define target GPS coordinates (latitude, longitude)
    points = []
    with open('/home/roisten/catkin_ws/recordings/14-05-2024-13-48.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, quotechar='|')
        for row in spamreader:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            points.append([x, y, z])
    target_gps = (points[0], points[1])
    #print(pid_controller.find_nearest_point(points))

    last = points[0]
    curr = points[1]
    heading = pid_controller.calculate_distance_and_heading(target_gps)
    print("heading: ", heading)
    # Define a tolerance level for considering the robot reached the target
    tolerance = 0.0001  # Adjust this based on the precision required

    
    # Proportional control gain for angular velocity
    k_p = 0.5  # Adjust this gain as needed
    # Start navigation
    #pid_controller.navigate_to_target(target_gps) # type: ignore

    rospy.init_node('listener', anonymous=True)
    
    rospy.spin()
