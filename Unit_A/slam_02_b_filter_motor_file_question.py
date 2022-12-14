# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    l, r = motor_ticks
    l *= ticks_to_mm; r *= ticks_to_mm
    x, y, theta = old_pose
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.

        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        theta_new = theta
        x_new = x + l*cos(theta_new)
        y_new = y + l*sin(theta_new)
        # --->>> Implement your code to compute x, y, theta here.
        return (x_new, y_new, theta_new)

    else:
        # Turn. Compute alpha, R, etc.

        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        
        # First
        x = x - scanner_displacement*cos(theta)
        y = y - scanner_displacement*sin(theta)

        # Second (copied from previous .py file)
        alpha = (r-l) / robot_width
        R = l / alpha
        C_x = x - (R + robot_width/2)*sin(theta)
        C_y = y - (R + robot_width/2)*-cos(theta)
        theta_new = (theta + alpha) % (2*pi)
        x_new = C_x + (R+robot_width/2)*sin(theta_new)
        y_new = C_y + (R+robot_width/2)*-cos(theta_new)

        # Third
        x_new = x_new + scanner_displacement*cos(theta_new)
        y_new = y_new + scanner_displacement*sin(theta_new)

        return (x_new, y_new, theta_new)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
