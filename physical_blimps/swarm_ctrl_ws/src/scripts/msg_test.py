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
        print(data.transform.translation)
        raise Exception('here')

    def talker():
        pub = rospy.Publisher('chatter', String, queue_size=10)
        print('here')
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        print('here')
        while not rospy.is_shutdown():
            # hello_str=TransformStamped()
            hello_str = "hello world %s" % rospy.get_time()
            pub.publish(hello_str)
            rate.sleep()
    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        print("here")
        rospy.init_node('listener', anonymous=True)
        print("heere")

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
