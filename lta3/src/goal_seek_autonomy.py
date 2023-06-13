#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseStamped
from lta3.msg import state_18DOF
from termcolor import colored
import sys
import copy
import math
import Vector3

class Waypoints:
    waypoint_publisher = None

    def __init__(self):
        self.waypoint_publisher = rospy.Publisher("ref", state_18DOF, queue_size=10)
        self.state_subscriber = rospy.Subscriber("state", state_18DOF, self.state_update)
        self.state = state_18DOF()

    def talker(self):
        print rospy.get_namespace() + "waypoint"
        while True:
            print "New Waypoint:"
            waypoint = raw_input()

            if waypoint == "q":
                sys.exit()

            v = state_18DOF()
            speed = 0.25
            weight = [1.0, 0]
            obstacle = [2.0, 0, 1.0]
            obs_range = 3
            obs_rep = 10

            try:
                coords = map(float, waypoint.split())
                print coords
                # obstacle = map(float, obstacle.split())

                v_goal = Vector3.Vector3(coords[0] - self.state.pose.pose.position.x,
                        coords[1] - self.state.pose.pose.position.y,
                        coords[2] - self.state.pose.pose.position.z)

                v_obs = Vector3.Vector3(obstacle[0] - self.state.pose.pose.position.x,
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

                if v_full.norm() > speed:
                    v_full.normalize()
                    v_full *= speed

                if len(coords) == 3:
                    v.header.frame_id = "/world"
                    v.velocity.twist.linear.x  = v_goal.x
                    v.velocity.twist.linear.y  = v_goal.y
                    v.velocity.twist.linear.z  = v_goal.z
                    v.pose.pose.orientation.w  = 1.0
                    #rospy.loginfo(v)
                    self.waypoint_publisher.publish(v)
                else:
                    print colored("Need 3 input coordinates.", "red")
            except:
                print colored("Bad input.", "red")

    def state_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.state = copy.deepcopy(data)


if __name__ == '__main__':
    rospy.init_node("waypointstestt")
    waypointstest = Waypoints()
    waypointstest.talker()
