#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, Int16, Float64
from termcolor import colored
from lta3.msg import floats
import time
import math

class PingRead(object):

    def __init__(self):
        self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        self.raw_state_subscriber = rospy.Subscriber("raw_state", floats, self.pingRead)
        self.ping_publisher = rospy.Publisher("altitude_ping", Float64, queue_size=10)

    def pingRead(self,data):
        """
        Parses raw state data for the altitude ping sensor

        :param data: NDOF pose estimate for LTA3
        :type data: Float64
        """
        ping = data.data[10]
        ping = ping/1000.0 #Convert from mm to m.
        self.ping_publisher.publish(ping)
