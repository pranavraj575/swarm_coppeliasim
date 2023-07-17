#!/usr/bin/env python2.7
from lta3.msg import state_18DOF
import tf
import rospy
from termcolor import colored


oldtime = 0.0
newtime = 0.0
old_pose = [0.0,0.0,0.0]
new_pose = [0.0,0.0,0.0]

def MUX(old_state,states):
    """
    Uses Vicon for pose estimate, and does not use a filter.  Also takes in the previous vicon_message to give velocity estimates

    :param old_vicon: The previous vicon message recieved for estimating velocity
    :type old_vicon: TransformStamped
    :param states: 6DOF state estimate for LTA3
    :type states: state_18DOF
    """

    state = state_18DOF()

    state.child_frame_id = "vicon_nofilter"
    state.header.stamp = states[0][1].header.stamp

    state.pose.pose.position.x = states[0][1].transform.translation.x
    state.pose.pose.position.y = states[0][1].transform.translation.y
    state.pose.pose.position.z = states[0][1].transform.translation.z

    #Switch from Quaternion to Euler
    pitch,roll,yaw = tf.transformations.euler_from_quaternion((states[0][1].transform.rotation.x,
                                                                states[0][1].transform.rotation.y,
                                                                states[0][1].transform.rotation.z,
                                                                states[0][1].transform.rotation.w))

    oldpitch,oldroll,oldyaw = tf.transformations.euler_from_quaternion((old_state.pose.pose.orientation.x,
                                                                old_state.pose.pose.orientation.y,
                                                                old_state.pose.pose.orientation.z,
                                                                old_state.pose.pose.orientation.w))

    state.pose.pose.orientation.x = states[0][1].transform.rotation.x
    state.pose.pose.orientation.y = states[0][1].transform.rotation.y
    state.pose.pose.orientation.z = states[0][1].transform.rotation.z
    state.pose.pose.orientation.w = states[0][1].transform.rotation.w


    #velocity estimate from pose data
    newtime = states[0][1].header.stamp.to_nsec()
    oldtime = old_state.header.stamp.to_nsec()
    dt = (newtime-oldtime)/1000000000.0
    print "new_time " + str(newtime)
    print "old_time " + str(oldtime)
    print "dt:      " + str(dt)

    new_pose = [state.pose.pose.position.x,state.pose.pose.position.y,state.pose.pose.position.z,pitch,roll,yaw]

    #The angular velocity might not be right
    old_state = [old_state.pose.pose.position.x,old_state.pose.pose.position.y,old_state.pose.pose.position.z,oldpitch,oldroll,oldyaw]

    try:
        state.velocity.twist.linear.x = (new_pose[0]-old_state[0])/dt
        state.velocity.twist.linear.y = (new_pose[1]-old_state[1])/dt
        state.velocity.twist.linear.z = (new_pose[2]-old_state[2])/dt

        state.velocity.twist.angular.x = (new_pose[3]-old_state[3])/dt
        state.velocity.twist.angular.y = (new_pose[4]-old_state[4])/dt
        state.velocity.twist.angular.z = (new_pose[5]-old_state[5])/dt

    except:
        print colored("Matching Vicon Timestamps. Using previous state...","red")
        state.velocity.twist.linear.x = old_state[0]
        state.velocity.twist.linear.y = old_state[1]
        state.velocity.twist.linear.z = old_state[2]

        state.velocity.twist.angular.x = old_state[3]
        state.velocity.twist.angular.y = old_state[4]
        state.velocity.twist.angular.z = old_state[5]



    '''
    state.velocity.twist.linear.x = -0.0
    state.velocity.twist.linear.y = -0.0
    state.velocity.twist.linear.z = -0.0

    state.velocity.twist.angular.x = -0.0
    state.velocity.twist.angular.y = -0.0
    state.velocity.twist.angular.z = -0.0
    '''

    state.acceleration.twist.linear.x = -0.0
    state.acceleration.twist.linear.y = -0.0
    state.acceleration.twist.linear.z = -0.0

    state.acceleration.twist.angular.x = -0.0
    state.acceleration.twist.angular.y = -0.0
    state.acceleration.twist.angular.z = -0.0

    oldtime = newtime

    return state
