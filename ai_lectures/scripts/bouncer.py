#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import math

#####################
# BEGIN Global Variable Definitions
obstacle_near = False
# END Global Variable Definitions
#####################+

#####################
# BEGIN ROS Topic Callback Function [DON'T MESS WITH THIS STUFF]
#####################

def laserCallback(data):
#This function will set a variable that tells us whether or not something is
#near the robot (in front)
    global obstacle_near

    obstacle_distance_threshold = 0.5

    min_range = data.range_max
    for i in range(len(data.ranges)):
        if data.ranges[i] < min_range:
            min_range = data.ranges[i]

    if min_range < obstacle_distance_threshold:
        obstacle_near = True
    else:
        obstacle_near = False

###########################
##### END CALLBACK FUNCTION  
###########################

class Robot(object):
    def __init__(self):
        self.state = NoObstacleDetected(self)

    def get_cmd_vel(self):
        return self.state.get_cmd_vel()

class RobotState(object):
    def __init__(self, robot):
        self.robot = robot

    def get_cmd_vel(self):
        raise NotImplementedError()

class NoObstacleDetected(RobotState):
    def __init__(self, robot):
        super(NoObstacleDetected, self).__init__(robot)
        self.command = Twist()
        self.command.linear.x = 0.5

    def get_cmd_vel(self):
        global obstacle_near
        if obstacle_near:
            robot.state = ObstacleDetected(robot)
        return self.command

class ObstacleDetected(RobotState):
    def __init__(self, robot):
        super(ObstacleDetected, self).__init__(robot)
        self.command = Twist()
        self.command.angular.z = 0.5

    def get_cmd_vel(self):
        global obstacle_near
        if not obstacle_near:
            robot.state = NoObstacleDetected(robot)
        return self.command

if __name__ == '__main__':
    rospy.init_node('bouncer', anonymous=True) #Initialize ROS Node
    pub = rospy.Publisher('cmd_vel', Twist) #Create publisher
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to laser scan
    rate = rospy.Rate(10)

    #SENSOR VARIABLE
    global obstacle_near  #Something is near the front of the robot (True/False)

    #########################################
    # PE 3 VARIABLE DECLARATION CODE : BEGIN
    #########################################

    robot = Robot()

    #######################################
    # PE 3 VARIABLE DECLARATION CODE : END
    #######################################

    while not rospy.is_shutdown():
        
        #Declare the twist command that we will publish this time step
        twist = Twist()
        
        ###############################
        # PE 3 INNER LOOP CODE : BEGIN
        ###############################

        twist = robot.get_cmd_vel()

        #############################
        # PE 3 INNER LOOP CODE : END
        #############################

        pub.publish(twist)
        rate.sleep()


