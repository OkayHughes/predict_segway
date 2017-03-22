#!/usr/bin/env python
import rospy
from pedes_predict.msg import Frame
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs import header
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import cm
import numpy as np
import time
plt.ion()
dic = {}

cmap = cm.viridis
varidis = cmap(np.arange(cmap.N))
varidis[:,-1] = np.linspace(0, 1, cmap.N)
varidis = ListedColormap(varidis)

width = 50

def callback(frame):
    global dic
    name = frame.agent_name
    if name not in dat.keys():
        dic[name] = []

    dic[name].append(frame)
    dic[name] = sorted(dic[name], key=lambda x: x.time)
    rospy.loginfo(len(dic[name]))

def display():
    global dic, plots
    reps = {}
    for key in dic.keys():
        dat = dic[key]
        dt = time.time()
        if len(dat) == 0:
            return
        else:
            while dat[0] < time.time():
                dat.pop(0)
                if len(dat) == 0:
                    return
                
        reps[key] = dat.pop(0)
    
    data = np.array(x.weights).reshape(width, width)
    delay = 0.9 * (x.time-time.time())

    if delay >= 0:
        time.sleep(0.9 * (x.time-time.time()))
    #data = np.ma.masked_array(data, data < np.amax(data)/10)
    


    
    dt = time.time()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('processor', anonymous=True)

    rospy.Subscriber("forecast/output", Frame, callback)
    rospy.loginfo("Started Listening")
    rate = rospy.Rate(100)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        #for key in dat.keys():
        #    litmus = sum([x == 0 for x in dat[key]])
        #    print "Key {}: {}".format(key, litmus)
        #    if litmus == 0:
        #        display(key)
        display()

if __name__ == '__main__':
    listener()
