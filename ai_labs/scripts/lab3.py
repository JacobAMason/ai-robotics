#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
import random

##########################
# BEGIN Global Variable Definitions
front_distance = -1.0
wall_distance = -1.0
# END Global Variable Definitions
##########################

##########################
# BEGIN ROS Topic Callback Function [DON'T MESS WITH THIS STUFF]
##########################
def laserCallback(laser_data):
#This function will set front_distance and wall_distance.  Both of these work in the same way.  If something is closer than the detection_distance for that sensor, the value will tell how close something is.  If nothing is closer than the detection distance, the value of -1 will be used.

    global front_distance
    global wall_distance

    front_detection_distance = 1.0
    wall_detection_distance = 0.9

    wf_min_right = 10.0
    wf_min_front = 10.0
    
    cur_angle = laser_data.angle_min 

    for i in range(len(laser_data.ranges)):
        if abs(cur_angle + math.pi/2) < (math.pi / 8):
            #Wall sensor ((-5/8)*math.pi <= cur_angle <= (-3/8)*math.pi)
            if laser_data.ranges[i] < wf_min_right:
                wf_min_right = laser_data.ranges[i]

        if abs(cur_angle) < (math.pi / 8):
            #Front sensor ((-1/8)*math.pi <= cur_angle <= (1/8)*math.pi)
            if laser_data.ranges[i] < wf_min_front:
                wf_min_front = laser_data.ranges[i]

        cur_angle = cur_angle + laser_data.angle_increment

    #Set the sensor variables
    front_distance = -1
    wall_distance = -1
    if wf_min_front < front_detection_distance:
        front_distance = wf_min_front
    if wf_min_right < wall_detection_distance:
        wall_distance = wf_min_right

############################
##### END CALLBACK FUNCTION   
############################

class Robot(object):
    def __init__(self):
        self.state = NoWallsVisible(self)

    def get_cmd_vel(self, front_distance, wall_distance):
        rospy.logdebug(self.state.__class__.__name__)
        return self.state.get_cmd_vel(front_distance, wall_distance)

class RobotState(object):
    def __init__(self, robot):
        self.robot = robot

    def get_cmd_vel(self, front_distance, wall_distance):
        raise NotImplementedError()

class NoWallsVisible(RobotState):
    def __init__(self, robot):
        super(NoWallsVisible, self).__init__(robot)
        self.command = Twist()
        self.command.linear.x = 0.5
        self.command.angular.z = -0.2

    def get_cmd_vel(self, front_distance, wall_distance):
        if front_distance == -1 and wall_distance == -1:
            robot.state = NoWallsVisible(robot)
        elif front_distance == -1:
            robot.state = WallToSide(robot)
        elif wall_distance == -1:
            robot.state = WallInFront(robot)
        else:
            robot.state = WallsInFrontAndSide(robot)
        return self.command

class RoundingCorner(RobotState):
    def __init__(self, robot):
        super(RoundingCorner, self).__init__(robot)
        self.command = Twist()
        self.command.linear.x = 0.1
        self.command.angular.z = -1.0

    def get_cmd_vel(self, front_distance, wall_distance):
        if front_distance == -1 and wall_distance == -1:
            robot.state = RoundingCorner(robot)
        elif front_distance == -1:
            pass
            # robot.state = WallToSide(robot)
        elif wall_distance == -1:
            robot.state = WallInFront(robot)
        else:
            # Disallow this state transition
            # robot.state = WallsInFrontAndSide(robot)
            pass
        return self.command

class WallInFront(RobotState):
    def __init__(self, robot):
        super(WallInFront, self).__init__(robot)
        self.command = Twist()

    def get_cmd_vel(self, front_distance, wall_distance):
        if front_distance == -1 and wall_distance == -1:
            robot.state = RoundingCorner(robot)
        elif front_distance == -1:
            robot.state = WallToSide(robot)
        elif wall_distance == -1:
            robot.state = WallInFront(robot)
        else:
            robot.state = WallsInFrontAndSide(robot)
        self.command.angular.z = (1 - front_distance)
        self.command.linear.x = 0.7 * front_distance
        return self.command

class WallToSide(RobotState):
    def __init__(self, robot):
        super(WallToSide, self).__init__(robot)
        self.command = Twist()

    def get_cmd_vel(self, front_distance, wall_distance):
        if front_distance == -1 and wall_distance == -1:
            robot.state = RoundingCorner(robot)
        elif front_distance == -1:
            robot.state = WallToSide(robot)
        elif wall_distance == -1:
            robot.state = WallInFront(robot)
        elif front_distance < 0.5:
            robot.state = WallsInFrontAndSide(robot)

        self.command.angular.z = -math.copysign(9*(wall_distance-0.3)**2, wall_distance-0.3)
        self.command.linear.x = 0.7 * (1 - min(1, abs(self.command.angular.z)))
        return self.command

class WallsInFrontAndSide(RobotState):
    def __init__(self, robot):
        super(WallsInFrontAndSide, self).__init__(robot)
        self.command = Twist()

    def get_cmd_vel(self, front_distance, wall_distance):
        if front_distance == -1 and wall_distance == -1:
            robot.state = RoundingCorner(robot)
        elif front_distance == -1:
            robot.state = WallToSide(robot)
        elif wall_distance == -1:
            robot.state = WallInFront(robot)
        else:
            robot.state = WallsInFrontAndSide(robot)
        self.command.angular.z = 1.0
        if front_distance > 0.2:
            self.command.linear.x = front_distance-0.2
        else:
            self.command.linear.x = 0
        return self.command

        
if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True, log_level=rospy.DEBUG) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to the laser scan topic

    rate = rospy.Rate(10) #10 Hz  
    
    #SENSOR VARIABLES
    global front_distance #How close is something in front of the robot (-1 = nothing is close)
    global wall_distance  #How close is something on the right side of the robot (-1 = nothing is close)
        
    #########################################
    # LAB 3 VARIABLE DECLARATION CODE : BEGIN
    #########################################

    robot = Robot()

    #######################################
    # LAB 3 VARIABLE DECLARATION CODE : END
    #######################################
    
    while not rospy.is_shutdown():
              
        ###############################
        # LAB 3 INNER LOOP CODE : BEGIN
        ###############################

        twist = robot.get_cmd_vel(front_distance, wall_distance)

        #############################
        # LAB 3 INNER LOOP CODE : END
        #############################
    
        #Publish drive command
        pub.publish(twist)
        rate.sleep() #Sleep until the next time to publish
