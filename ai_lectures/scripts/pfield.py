#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math

# Global Variable for robot pose
robot = [0,0,0]

# Robot Position Callback function
def robotCallback(data):
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x, data.pose.pose.position.y, yaw]

# Helper functions

# This helper function adds two forces together and returns the result
def add_forces(a, b):
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c
    
# This helper function converts input angle to one -pi to pi
def wrap_angle(angle):
    while angle >= math.pi:
        angle = angle - 2*math.pi
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

# This function takes in a (global coordinates) force and returns the drive command to publish
def drive_from_force(force):
    global robot #format [x_pos, y_pos, yaw]
    twist = Twist()

    twist.angular.z = wrap_angle(math.atan2(force[1], force[0]) - robot[2])

    # Don't move forward unless the robot's heading is within 45 degrees of the goal
    if abs(twist.angular.z) < math.pi/4:
        twist.linear.x = math.hypot(force[0], force[1])
    
    return twist

def goal_force():
    global robot  #[x, y, yaw]
    goal = [-6, 6]
    multiplier = 3
    
    return [mag * multiplier for mag in (goal[0] - robot[0], goal[1] - robot[1])]
    
if __name__ == '__main__':
    #ROS Stuff
    rospy.init_node('PE2', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("base_pose_ground_truth", Odometry, robotCallback)
    rate = rospy.Rate(10) # 10hz

    #Main loop
    while not rospy.is_shutdown():
       
        # 1. Influence robot with Uniform Force (global coordinates)
        uniform_force = [1,1]
        
        # 2. Get the attractive force to goal
        g_force = goal_force()
        
        # 3. Add the two forces together
        total_force = add_forces(uniform_force, g_force)
        
        # 4. Get the drive command for this force
        twist = drive_from_force(total_force)

        # 5. Publish the command
        pub.publish(twist)
        
        # 6. Sleep until it is time to do it again
        rate.sleep()


