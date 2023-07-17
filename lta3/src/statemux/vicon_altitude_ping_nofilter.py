#!/usr/bin/env python2.7
from lta3.msg import state_18DOF
import tf
import rospy



def MUX(states):
    """
    Uses Vicon and altitude ping sensor for pose estimate, and does not use a filter.  The altitude ping sensor overrirdes the Vicon altitude reading.

    :param states: 6DOF state estimate for LTA3
    :type states: state_18DOF
    """
    state = state_18DOF()

    state.child_frame_id = "vicon_altitude_ping_nofilter"
    state.header.stamp = rospy.Time.now()

    state.pose.pose.position.x = states[0][1].transform.translation.x
    state.pose.pose.position.y = states[0][1].transform.translation.y
    #Retrieve altitude from ping sensor instead of vicon
    #print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " + str(states[2].data)
    state.pose.pose.position.z = states[2].data

    #Switch from Quaternion to Euler
    pitch,roll,yaw = tf.transformations.euler_from_quaternion((states[0][1].transform.rotation.x,
                                                                states[0][1].transform.rotation.y,
                                                                states[0][1].transform.rotation.z,
                                                                states[0][1].transform.rotation.w))

    state.pose.pose.orientation.x = states[0][1].transform.rotation.x
    state.pose.pose.orientation.y = states[0][1].transform.rotation.y
    state.pose.pose.orientation.z = states[0][1].transform.rotation.z
    state.pose.pose.orientation.w = states[0][1].transform.rotation.w



    state.velocity.twist.linear.x = -0.0
    state.velocity.twist.linear.y = -0.0
    state.velocity.twist.linear.z = -0.0

    state.velocity.twist.angular.x = -0.0
    state.velocity.twist.angular.y = -0.0
    state.velocity.twist.angular.z = -0.0

    state.acceleration.twist.linear.x = -0.0
    state.acceleration.twist.linear.y = -0.0
    state.acceleration.twist.linear.z = -0.0

    state.acceleration.twist.angular.x = -0.0
    state.acceleration.twist.angular.y = -0.0
    state.acceleration.twist.angular.z = -0.0

    return state
