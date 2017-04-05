#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
from math import *

color_center = -1.0
obstacle_near = False


def drive_from_state(twist):
    global obstacle_near #True or False
    global color_center #-1 if no color detected, otherwise between 0.0 and 1.0, with 0.0 
    #meaning on the left, 1.0 on the right (of camera's field of view), and 0.5 meaning straight ahead. 

    #CODE HERE TO CONTROL ROBOT TO SEEK GREEN AND AVOID OBSTACLES

    return twist

def laserCallback(data):
    global obstacle_near

    obstacle_distance_threshold = 1.0

    print 'Got new laser scan at ', rospy.Time.now()
    min_range = data.range_max
    for i in range(len(data.ranges)):
        if data.ranges[i] < min_range:
            min_range = data.ranges[i]

    print 'Minimum range from scan is : ', min_range
    if min_range < obstacle_distance_threshold:
        obstacle_near = True
    else:
        obstacle_near = False

def rgbTohsv(ri,gi,bi):
    #Normalize
    r = ri / 255.0
    g = gi / 255.0
    b = bi / 255.0
    
    var_Min = min(r, g, b)
    var_Max = max(r, g, b)
    del_Max = var_Max - var_Min

    v = var_Max

    if del_Max == 0:
        h = 0
        s = 0
    else:
        s = del_Max / var_Max
        
        del_R = ( ( ( var_Max - r ) / 6 ) + ( del_Max / 2 ) ) / del_Max
        del_G = ( ( ( var_Max - g ) / 6 ) + ( del_Max / 2 ) ) / del_Max
        del_B = ( ( ( var_Max - b ) / 6 ) + ( del_Max / 2 ) ) / del_Max
        
        if r == var_Max:
            h = del_B - del_G
        elif g == var_Max:
            h = (1.0 / 3.0) + del_R - del_B
        elif b == var_Max:
            h = (2.0 / 3.0) + del_G - del_R

        if h < 0:
            h = h + 1.0
        if h > 1:
            h = h - 1.0

    return [360.0*h,s,v]

def cameraCallback(data):
    global color_center
    
    mass = 0.0
    num = 0.0


    for j in range(data.height):
        for i in range(data.width):
            bi = j*data.width*4 + i*4 #4 comes from RGBA format
            [h,s,v] = rgbTohsv(ord(data.data[bi]),ord(data.data[bi+1]),ord(data.data[bi+2]))
            if h > 110.0 and h < 150.0:
                #Green block
                mass = mass + i
                num = num + 1

    #Compute center of green mass
    color_center = -1
    if num > 0:
        color_center = (mass / num) / (data.width - 1 )
        print 'Mass = ', mass
        print 'Num = ', num
    print 'Center of color mass is at: ', color_center

if __name__ == '__main__':
    rospy.init_node('rgb', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to the laser scan topic
    rospy.Subscriber("image", Image, cameraCallback) #Subscribe to the image topic

    while not rospy.is_shutdown():
        twist = Twist()
        
        #Get final drive command from total force
        twist = drive_from_state(twist) 

        #Publish drive command, then sleep
        pub.publish(twist)
        rospy.sleep(0.1)
        

    print 'Done'

    twist = Twist()
    pub.publish(twist)

