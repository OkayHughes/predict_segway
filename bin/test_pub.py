#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from pedes_predict.msg import Observation
import time
from numpy.random import uniform
from numpy import pi

def new_rand(name):
    obs = Observation()
    prt = time.time()
    state = Pose2D()
    state.x = uniform(-1, 1)
    state.y = uniform(-1, 1)
    state.theta = uniform(0, 2 * pi)
    obs.pose = state
    obs.time = rospy.get_time()
    obs.agent_name = name
    return obs

def talker():
    pub = rospy.Publisher('forecast/input', Observation, queue_size=10)
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    prt = time.time()
    rospy.loginfo("Started Publisher")
    while not rospy.is_shutdown():
        if time.time() - prt < .2:
            rate.sleep()
            continue
        
        rospy.loginfo("Published")
        pub.publish(new_rand("0"))
        pub.publish(new_rand("1"))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
