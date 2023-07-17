#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3, PoseStamped
from lta3.msg import state_18DOF, floats
from termcolor import colored
import sys
import copy
import math
import Vector3f

class Waypoints:
    waypoint_publisher = None

    def __init__(self):
        self.lta3_id = rospy.get_param('~lta3_id', "lta0")

        self.waypoint_publisher = rospy.Publisher("ref", state_18DOF, queue_size=10)
        self.data_publisher = rospy.Publisher("data", floats, queue_size=10)

        self.wp_subscriber = rospy.Subscriber("ref_wp", state_18DOF, self.wp_update)
        self.wp = state_18DOF()

        self.state_subscriber = rospy.Subscriber("state", state_18DOF, self.state_update)
        self.state = state_18DOF()


    def talker(self):
        height = 1
        waypointf = Vector3f.Vector3(1, 0, height)
        weight = [1.0, 0]
        obstacle = [-3, 0, height]
        max_speed = 1.25
        obs_range = 3
        obs_rep = 100
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            v_goal = Vector3f.Vector3(self.wp.pose.pose.position.x - self.state.pose.pose.position.x,
                self.wp.pose.pose.position.y - self.state.pose.pose.position.y,
                self.wp.pose.pose.position.z - self.state.pose.pose.position.z)

            v_obs = Vector3f.Vector3(obstacle[0] - self.state.pose.pose.position.x,
                obstacle[1] - self.state.pose.pose.position.y,
                obstacle[2] - self.state.pose.pose.position.z)

            dist = v_obs.norm()
            if dist < obs_range:
                v_obs.normalize();
                if dist != 0:
                    v_obs *= (-1 * (obs_rep * 1 * 1) / (dist ** 2));
                else:
                    v_obs *= (-1 * (obs_rep * 1 * 1) / 0.00001);
            else:
                v_obs.zero()

            v_full = v_goal * weight[0] + v_obs * weight[1]

            if v_full.norm() > max_speed:
                v_full.normalize()
                v_full *= max_speed

            p_full = Vector3f.Vector3(v_full.x + self.state.pose.pose.position.x,
                v_full.y + self.state.pose.pose.position.y,
                v_full.z + self.state.pose.pose.position.z)

            # print self.lta3_id + "   " + str(self.wp.pose.pose.position)
            # print "v_goal: " + str(v_goal.norm())
            # print "x_obs:  " + str(v_obs.norm())
            # print "v_full: " + str(v_full.norm())

            v = state_18DOF()

            v.header.frame_id = "/world"
            v.pose.pose.position.x  = p_full.x
            v.pose.pose.position.y  = p_full.y
            v.pose.pose.position.z  = p_full.z
            v.pose.pose.orientation.w  = 1.0
            self.waypoint_publisher.publish(v)

            w = floats()
            w = [float(self.lta3_id.replace("lta", "")),
                self.state.pose.pose.position.x, 
                self.state.pose.pose.position.y,
                self.state.pose.pose.position.z]
            self.data_publisher.publish(w)

            rate.sleep()


    def state_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.state = copy.deepcopy(data)

    def wp_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.wp = copy.deepcopy(data)


if __name__ == '__main__':
    rospy.init_node("waypointstest")
    waypointstest = Waypoints()
    waypointstest.talker()
