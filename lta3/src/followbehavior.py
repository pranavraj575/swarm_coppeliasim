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
        self.prey = rospy.get_param('~prey', "lta1")

        self.waypoint_publisher = rospy.Publisher("ref", state_18DOF, queue_size=10)

        self.state_subscriber = rospy.Subscriber("state", state_18DOF, self.state_update)
        self.state = state_18DOF()

        self.data_subscriber = rospy.Subscriber("data", floats, self.data_update)
        self.data = floats()


    def talker(self):
        height = 1
        waypointf = Vector3f.Vector3(1, 0, height)
        weight = [1.0]
        max_speed = 1.25
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            if not self.data.data:
                v_goal = Vector3f.Vector3(0, 0, 0)
            elif float(self.prey.replace("lta", "")) == self.data.data[0]:
                v_goal = Vector3f.Vector3(self.data.data[1] - self.state.pose.pose.position.x,
                    self.data.data[2] - self.state.pose.pose.position.y,
                    self.data.data[3] - self.state.pose.pose.position.z)

            v_full = v_goal * weight[0]

            if v_full.norm() > max_speed:
                v_full.normalize()
                v_full *= max_speed

            p_full = Vector3f.Vector3(v_full.x + self.state.pose.pose.position.x,
                v_full.y + self.state.pose.pose.position.y,
                v_full.z + self.state.pose.pose.position.z)

            v = state_18DOF()
            print str(p_full)
            v.header.frame_id = "/world"
            v.pose.pose.position.x  = p_full.x
            v.pose.pose.position.y  = p_full.y
            v.pose.pose.position.z  = p_full.z
            v.pose.pose.orientation.w  = 1.0
            self.waypoint_publisher.publish(v)

            rate.sleep()


    def state_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.state = copy.deepcopy(data)

    def data_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.data = copy.deepcopy(data)


if __name__ == '__main__':
    rospy.init_node("waypointstest")
    waypointstest = Waypoints()
    waypointstest.talker()
