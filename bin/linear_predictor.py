#! /usr/bin/env python

import rospy
from pedes_predict.msg import Frame
from geometry_msgs.msg import Point, Pose2D
from predict.code.generate_distributions import linear_generator
import numpy as np
from multiprocessing import Process, Queue
import time



    

def talker(queue):
    pub = rospy.Publisher('forecast/output', Frame, queue_size=10)
    rospy.init_node('forecast_write', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    gen = None

    while not rospy.is_shutdown():
        while 1:
            fr = queue.get()
            if not fr:
                break
            pub.publish(fr)
            rospy.loginfo("Agent: {} Frame {}".format(fr.agent_name, fr.frame))
        rate.sleep()


def calculate(generator, queue, num):
    frame = 0
    tm = time.time()
    res = [x for x in generator]
    rospy.loginfo(time.time() - tm)

    for pt in res:
          fr = Frame()
          fr.frame = frame
          fr.agent_name= str(num)
          arr = pt[0].astype(float).tolist()
          fr.xs = arr[0]
          fr.ys = arr[1]
          fr.weights = pt[1].astype(float).tolist()
          queue.put(fr)
          frame += 1

num = 0
def listener(queue):
    def callback(data):
        global num
        x = np.array([data.x, data.y])
        v = np.array([np.cos(data.theta)/200, np.sin(data.theta)/200])
        rospy.loginfo(x)
        rospy.loginfo(v)
        t_final = 300
        n_steps = 50
        mag = np.sqrt(v[0]**2 + v[1] ** 2)
        width = 3.0
        Process(target = calculate, args=(linear_generator(x, v, t_final, n_steps, width), queue, num)).start()
        num += 1

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('forecast_read', anonymous=True)
    rospy.Subscriber("forecast/input", Pose2D, callback)
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
