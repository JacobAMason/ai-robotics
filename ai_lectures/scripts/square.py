#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from itertools import cycle


def square():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('square', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    forwardTwist = Twist()
    forwardTwist.linear.x = 0.7
    turnLeftTwist = Twist()
    turnLeftTwist.angular.z = 0.75
    get_twist_command = cycle((forwardTwist, turnLeftTwist)).next

    print 'Starting square . . .'

    seconds = rospy.get_time()
    twistCommand = get_twist_command()
    while not rospy.is_shutdown():
        if rospy.get_time() > 2 + seconds:
            seconds = rospy.get_time()
            twistCommand = get_twist_command()
        pub.publish(twistCommand)
        rate.sleep() #sleep until the next time to publish

    print 'Done.'

if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass
