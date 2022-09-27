# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from math import sin, cos, pi
from pylab import *
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    l, r = motor_ticks
    l *= ticks_to_mm; r *= ticks_to_mm
    x, y, theta = old_pose
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        theta_new = theta
        x_new = x + l*cos(theta_new)
        y_new = y + l*sin(theta_new)
        # --->>> Implement your code to compute x, y, theta here.
        return (x_new, y_new, theta_new)

    else:
        # Turn. Compute alpha, R, etc.
        alpha = (r-l) / robot_width
        R = l / alpha
        C_x = x - (R + robot_width/2)*sin(theta)
        C_y = y - (R + robot_width/2)*-cos(theta)
        theta_new = (theta + alpha) % (2*pi)
        x_new = C_x + (R+robot_width/2)*sin(theta_new)
        y_new = C_y + (R+robot_width/2)*-cos(theta_new)
        # --->>> Implement your code to compute x, y, theta here.
        return (x_new, y_new, theta_new)

if __name__ == '__main__':
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0 # Distance between the two caterpillar tracks

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), looking along x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    # Draw result.
    for pose in filtered:
        print pose
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()
