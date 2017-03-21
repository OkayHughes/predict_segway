#!/usr/bin/env python
import rospy
from pedes_predict.msg import Frame
import matplotlib.pyplot as plt
import numpy as np
plt.ion()

dat = {}
def callback(frame):
    if frame.agent_name not in dat.keys() and len(dat.keys()) < 2:
        dat[frame.agent_name] = [0 for x in range(50)]
    if frame.agent_name in dat.keys():
        dat[frame.agent_name][frame.frame] = ([frame.xs, frame.ys], frame.weights)

def display(key):
    global dat
    data = dat[key]

    for x in data:
        plt.scatter(x[0][0], x[0][1], c=x[1], cmap="viridis", edgecolors="none")
        print np.average(data[x][1])
        plt.pause(0.05)
        dat = {}
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('processor', anonymous=True)

    rospy.Subscriber("forecast/output", Frame, callback)
    rate = rospy.Rate(10)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        for key in dat.keys():
            litmus = sum([x == 0 for x in dat[key]])
            print "Key {}: {}".format(key, litmus)
            if litmus == 0:
                display(key)
        rate.sleep()

if __name__ == '__main__':
    listener()
