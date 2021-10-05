#!/usr/bin/env python
import rospy
from math import pi, fmod, sin, cos, sqrt
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_path.srv import *
# hint: some imports are missing

cur_pos = Pose()

def cb_pose(data): # get the current position from subscribing the turtle position
    global cur_pos
    cur_pos = data

def cb_walk(req):
    if (req.distance < 0):
        return False

    # hint: calculate the projected (x, y) after walking the distance,
    # and return false if it is outside the boundary
    angle = cur_pos.theta
    desX = req.distance * cos(angle) + cur_pos.x
    desY = req.distance * sin(angle) + cur_pos.y

    if desX < 0 or desX > 11 or desY < 0 or desY > 11:
        return False

    rate = rospy.Rate(100) # 100Hz control loop

    Distance = sqrt((desX - cur_pos.x) ** 2 + (desY - cur_pos.y) ** 2)
    while (Distance != 0): # control loop
        
        # in each iteration of the control loop, publish a velocity
        vel = Twist()
        vel.linear.x = Distance
        pub.publish(vel)
        # hint: you need to use the formula for distance between two points
        
        Distance = sqrt((desX - cur_pos.x) ** 2 + (desY - cur_pos.y) ** 2)

        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    vel.linear.x = 0
    pub.publish(vel)

    return True

def cb_orientation(req):

    rate = rospy.Rate(100) # 100Hz control loop
    
    if req.orientation > pi or req.orientation < pi * -1:
        return False

    error = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi
    while (error != 0): # control loop
        
        # in each iteration of the control loop, publish a velocity
        vel = Twist()
        vel.angular.z = error
        pub.publish(vel)
        # hint: signed smallest distance between two angles: 
        # see https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
        error = fmod(req.orientation - cur_pos.theta + pi + 2 * pi, 2 * pi) - pi

        rate.sleep()
    
    vel = Twist() # publish a velocity 0 at the end, to ensure the turtle really stops
    vel.angular.z = 0
    pub.publish(vel)

    return True

if __name__ == '__main__':
    rospy.init_node('path_manager')
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1) # publisher of the turtle velocity
    sub = rospy.Subscriber('turtle1/pose', Pose, cb_pose) # subscriber of the turtle position, callback to cb_pose
    
    ## init each service server here:
    rospy.Service('set_orientation', SetOrientation, cb_orientation)		# callback to cb_orientation
    rospy.Service('walk_distance', WalkDistance, cb_walk)		# callback to cb_walk
    
    rospy.spin()
