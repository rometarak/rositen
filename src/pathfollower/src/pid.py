import time
import math
import csv
import numpy as np

# Sample PID controller implementation
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output

# GPS class
class GPS:
    def __init__(self):
        self.current_latitude = 0
        self.current_longitude = 0

    def get_coordinates(self):
        points = []
        with open('Point.csv', newline='') as csvfile:
            reader = csv.reader(csvfile, quotechar='|')
            for row in reader:
                x = row[0]
                y = row[1]
                points.append([x,y])

# Motor class
class Motor:
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0

    def set_speed(self, left_speed, right_speed):
        self.left_speed = left_speed
        self.right_speed = right_speed
        
def main():
    # pid_controller
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0
    pid_controller = PIDController(Kp, Ki, Kd)

    gps = GPS()
    motor = Motor()

    desired_latitude = 1.0
    desired_longitude = 1.0

    while True:
        # get gps coordinates
        current_latitude, current_longitude = gps.get_coordinates()

        # calculate error between current and desired coordinates
        error_latitude = desired_latitude - current_latitude
        error_longitude = desired_longitude - current_longitude

        # calculate total error (distance)
        total_error = math.sqrt(error_latitude ** 2 + error_longitude ** 2)

        # Calculate control signal using PID controller
        control_signal = pid_controller.calculate(total_error, dt=1)  # dt is the time step

        # Adjust left and right motor speeds based on control signal
        left_speed = control_signal
        right_speed = control_signal

        # Apply control signal to motors
        motor.set_speed(left_speed, right_speed)

        time.sleep(0.1)

if __name__ == "__main__":
    main()

