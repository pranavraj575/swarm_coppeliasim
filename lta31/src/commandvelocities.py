#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3, PoseStamped
from lta3.msg import state_18DOF
from termcolor import colored
import sys


class Waypoints:
    waypoint_publisher = None

    def __init__(self):
        self.waypoint_publisher = rospy.Publisher("ref", state_18DOF, queue_size=10)

    def talker(self):
        print rospy.get_namespace() + "waypoint"
        while True:
            print "New Waypoint:"
            waypoint = raw_input()

            if waypoint == "q":
                sys.exit()

            v = state_18DOF()

            try:
                coords = map(float, waypoint.split())
                print coords

                if len(coords) == 3:
                    v.header.frame_id = "/world"
                    v.velocity.twist.linear.x  = coords[0]
                    v.velocity.twist.linear.y  = coords[1]
                    v.velocity.twist.linear.z  = coords[2]
                    v.pose.pose.orientation.w  = 1.0
                    #rospy.loginfo(v)
                    self.waypoint_publisher.publish(v)
                else:
                    print colored("Need 3 input coordinates.", "red")
            except:
                print colored("Bad input.", "red")


if __name__ == '__main__':
    rospy.init_node("waypointstestt")
    waypointstest = Waypoints()
    waypointstest.talker()
