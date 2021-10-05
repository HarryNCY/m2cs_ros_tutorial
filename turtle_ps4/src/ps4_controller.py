#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from m2_ps4.msg import Ps4Data
from turtlesim.srv import *
# hint: some imports are missing

old_data = Ps4Data()
speed = 2
vel = Twist()

# colors
green = SetPen()
green.r = 0
green.g = 255
green.b = 0
green.width = 3
green.off = 0

red = SetPen()
red.r = 255
red.g = 0
red.b = 0
red.width = 3
red.off = 0

blue = SetPen()
blue.r = 0
blue.g = 0
blue.b = 255
blue.width = 3
blue.off = 0

purple = SetPen()
purple.r = 148
purple.g = 0
purple.b = 211
purple.width = 3
purple.off = 0

def callback(data):
    global old_data
    global speed
    
    # basic driving
    global vel

    vel.linear.x = speed * data.hat_lx
    vel.angular.z = speed * data.hat_rx

    # velocity scale
    

    if (data.dpad_x > 0) and (old_data.dpad_x == 0):
        speed += 2
    elif (data.dpad_x < 0) and (old_data.dpad_x == 0):
        speed -= 2
    
    if speed < 0:
        speed = 0
    elif speed > 10:
        speed = 10

    pub.publish(vel)

    # you should publish the velocity here!
    
    # hint: to detect a button being pressed, you can use the following pseudocode:
    # 
    # if ((data.button is pressed) and (old_data.button not pressed)),
    # then do something...
    
    # Clear background
    if (data.ps) and (not old_data.ps):
        resp_clr = srv_clr()

    # Change pen color
    if (data.triangle) and (not old_data.triangle):
        resp_col = srv_col(green)
    if (data.circle) and (not old_data.circle):
        resp_col = srv_col(red)
    if (data.cross) and (not old_data.cross):
        resp_col = srv_col(blue)
    if (data.square) and (not old_data.square):
        resp_col = srv_col(purple)

    old_data = data

if __name__ == '__main__':
    rospy.init_node('ps4_controller')
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1) # publisher object goes here... hint: the topic type is Twist
    sub = rospy.Subscriber('input/ps4_data', Ps4Data, callback) # subscriber object goes here
    
    # one service object is needed for each service called!
    srv_col = rospy.ServiceProxy('turtle1/set_pen', SetPen) # service client object goes here... hint: the srv type is SetPen
    # fill in the other service client object...
    srv_clr = rospy.ServiceProxy('clear')
    
    rospy.spin()
