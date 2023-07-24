#!/usr/bin/env python3
import numpy as np
import time
from collections import deque
import rospy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import os, sys
from std_msgs.msg import String

if __name__=="__main__":
    
    def callback(data):
        print(data)

    def talker():
        pub = rospy.Publisher('chatter', TransformStamped, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str=TransformStamped()
            tim=time.time()
            hello_str.header.stamp.secs=int(tim)
            hello_str.header.stamp.nsecs=int(10**9*(tim-int(tim)))
            
            hello_str.transform.translation.x=tim%10000+np.random.random()*.01
            hello_str.transform.translation.y=tim%10000
            hello_str.transform.translation.z=tim%10000
            
            hello_str.transform.rotation.x=np.random.random()
            hello_str.transform.rotation.y=np.random.random()
            hello_str.transform.rotation.z=np.random.random()
            hello_str.transform.rotation.w=np.random.random()
            
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()
    
    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("chatter", TransformStamped, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    
    if '--send' in sys.argv or '-s' in sys.argv:
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
    elif '--receive' in sys.argv or '-r' in sys.argv:
        listener()
    else:
        print("Use with --send (-s) or --receive (-r)")
