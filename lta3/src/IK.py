#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import TwistStamped
#catkin_make must be called to build this custom message type
from lta3.msg import floats
from termcolor import colored
from helper import switch
import time

from mav1 import MAV1_IK


class InverseKinematics:
    """
    This class takes in an 18 DOF desired output vector and performs Inverse Kinematics to map the propper joint space motor outputs to each of the blimps
    """

    def __init__(self):
        self.blimp_type = rospy.get_param('~blimp_type', "MAV1")
        """Reads in the blimp type from the IK.launch parameter **blimp_type** """
        self.des_subscriber = rospy.Subscriber("des", TwistStamped, self.des_update)
        """Subscribes to 18 DOF des message for an individual LTA3 and activates joint space transcribing loop based of off the blimp_type"""
        self.js_publisher = rospy.Publisher("js", floats,  queue_size = 10)
        """Publishes joint space in an n-sized float array to the drivers array"""


    def des_update(self, data):
        """
        Converts 18 DOF desired vector into N-dimensional joint space.  This is blimp_type specific. This then immediately publishes the js message

        :param data: 18DOF desired vector for LTA3
        :type data: lta3.msg
        """

        js = MAV1_IK.setMove(data)

        self.js_publisher.publish(js)
        #print "publishing joint space"

if __name__ == '__main__':
    rospy.init_node("IK")
    IK = InverseKinematics()
    rospy.spin()
