#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
import time
from numpy.random import uniform
from numpy import pi

def talker():
    pub = rospy.Publisher('forecast/input', Pose2D, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    prt = time.time()
    while not rospy.is_shutdown():
        if time.time() - prt < 10:
            rate.sleep()
            continue
        prt = time.time()
        state = Pose2D()
        state.x = uniform(-1, 1)
        state.y = uniform(-1, 1)
        state.theta = uniform(0, 2 * pi) 
        rospy.loginfo("Published")
        pub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
