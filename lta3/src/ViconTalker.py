#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Example Python node to publish on a specific topic."""

# Import required Python code.
import rospy
from lta3.msg import state_18DOF
from geometry_msgs.msg import TransformStamped
import tf2_ros
import random


class State(object):
    """Node example class."""

    def __init__(self):
        """Read in parameters."""
        rate = rospy.get_param('~rate', 10.0)
        id = rospy.get_param('~id', 'lta1')
        self.state_pub = rospy.Publisher("vicon" + '/' + id + '/' + id, TransformStamped, queue_size=10)
        #self.ref_pub = rospy.Publisher("ref", state_18DOF, queue_size=10)
        rospy.Rate(rate) # 10hz

    def talker(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            '''
            tf = state_18DOF()
            tf.child_frame_id = "state"
            tf.header.stamp = rospy.Time.now()

            tf.pose.pose.position.x = randint(-10, 10)
            tf.pose.pose.position.y = randint(-10, 10)
            tf.pose.pose.position.z = randint(-10, 10)

            tf.pose.pose.orientation.x = 0.
            tf.pose.pose.orientation.y = 0.
            tf.pose.pose.orientation.z = 0.
            tf.pose.pose.orientation.w = 1.
            '''
            tf = TransformStamped()
            tf = TransformStamped()
            tf.child_frame_id = "VICON_STATIC"
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = round(random.uniform(-1.0,1.0), 2)
            tf.transform.translation.y = round(random.uniform(-1.0,1.0), 2)
            tf.transform.translation.z = round(random.uniform(-1.0,1.0), 2)

            tf.transform.rotation.x = round(random.uniform(-1.0,1.0), 2)
            tf.transform.rotation.y = round(random.uniform(-1.0,1.0), 2)
            tf.transform.rotation.z = round(random.uniform(-1.0,1.0), 2)
            tf.transform.rotation.w = round(random.uniform(-1.0,1.0), 2)

            self.state_pub.publish(tf)
            #self.ref_pub.publish(tf)
            rospy.loginfo("publishing state...")
            rate.sleep()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('state')
    # Go to class functions that do all the heavy lifting.
    try:
        State()
        State().talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
