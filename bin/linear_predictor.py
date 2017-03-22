#! /usr/bin/env python

import rospy
from pedes_predict.msg import Frame, Observation
from geometry_msgs.msg import Point, Pose2D
from predict.code.generate_distributions import linear_generator
import numpy as np
from multiprocessing import Process, Queue
import time



    

def talker(queue):
    pub = rospy.Publisher('forecast/output', Frame, queue_size=10)
    rospy.init_node('forecast_write', anonymous=True)
    rate = rospy.Rate(300) # 10hz
    gen = None
    rospy.loginfo("Started publisher")
    while not rospy.is_shutdown():
        while 1:
            fr = queue.get()
            if not fr:
                break
            pub.publish(fr)
            rospy.loginfo("Agent: {} Frame {}".format(fr.agent_name, fr.time))
        rate.sleep()




namet = {}
def listener(queue):
    def callback(data):
        name = data.agent_name
        if name not in namet.keys():
            namet[name] = rospy.get_time()
        elif namet[name] > rospy.get_time():
            return

        x = np.array([data.pose.x, data.pose.y])
        v = np.array([np.cos(data.pose.theta)/200, np.sin(data.pose.theta)/200])
        rospy.loginfo(x)
        rospy.loginfo(v)
        t_final = 300
        n_steps = 50

        namet[name] = data.time
        namet[name] += t_final/30.0

        mag = np.sqrt(v[0]**2 + v[1] ** 2)
        width = 3.0
        frame = 0

        generator = linear_generator(x, v, t_final, n_steps, width)
        res = [x for x in generator]

        for j, pt in enumerate(res):
            data.time += (1.0/n_steps) * (t_final / 30.0)
            fr = Frame()
            fr.time = data.time

            fr.agent_name= data.agent_name
            arr = pt[0].astype(float).tolist()
            fr.xs = arr[0]
            fr.ys = arr[1]
            fr.weights = pt[1].astype(float).tolist()
            queue.put(fr)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('forecast_read', anonymous=True)
    rospy.Subscriber("forecast/input", Observation, callback)
    rospy.loginfo("Started Listener")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        #x = np.array([0,0])
        #v = np.array([-1, 0])
        #t = 300
        #N_steps = 50
        #gen = linear_generator(x, v, t, N_steps, 3)
        #for x in gen:
        #    print x[0].shape
        #    print x[1].shape
        queue = Queue()
        p1 = Process(target=talker, args=(queue,))
        p1.start()
        p2 = Process(target=listener, args=(queue,))
        p2.start()
        p1.join()
        p2.join()
    except rospy.ROSInterruptException:
        pass
