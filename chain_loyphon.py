#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Vector3, PoseStamped
from lta3.msg import state_18DOF
from termcolor import colored
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import sys
import copy
import math
import Vector3f

import numpy as np


class Waypoints:
    waypoint_publisher = None

    def __init__(self):
        self.lta3_id = rospy.get_param('~lta3_id', "lta0")

        self.waypoint_publisher = rospy.Publisher("ref", state_18DOF, queue_size=10)

        self.wp_subscriber = rospy.Subscriber("ref_wp", state_18DOF, self.wp_update)
        self.wp = state_18DOF()

        self.state_subscriber = rospy.Subscriber("state", state_18DOF, self.state_update)
        self.state = state_18DOF()

        self.data_publisher = rospy.Publisher("data", Float32MultiArray, queue_size=10)
        self.data_subscriber = rospy.Subscriber("data", Float32MultiArray, self.data_update)
        self.data = Float32MultiArray()

    def talker(self):
        num_agents = 12
        datasize = 5
        height = 1
        # weight = [1.0, 0, 0, 0, 0]
        weight = [1.0, 1.0, 0.05, 20, 50]

        # obstacle = [[-1.5, -1.0, height],
        #             [0.0, 0.5, height],
        #             [0.0, -2.5, height]]
        obstacle = [[-1.5, 0.0, height],
                    [0.0, 1.5, height],
                    [0.0, -1.5, height]]
        initial_speed = .5
        max_speed = 0.75
        obs_range = 1.5
        obs_rep = 10
        etta = 0.4
        neighbor_range = 1.75
        radius = 0.5
        coll_range = 0.25
        goal_area = [-5, 0, height]
        goalx = -1.5
        goal_range = 2
        # workspace = [-5, 5, -1.5, 4]
        workspace = [-5, 5, -0.5, 4]

        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():

            idnum = float(self.lta3_id.replace("lta", ""))
            if idnum == float(101):
                idnum == 11
            if idnum == float(102):
                idnum == 12

            #####################################################################################################
            # Goal
            v_goal = Vector3f.Vector3(0, 0, 0)
            if self.wp:
                v_goal = Vector3f.Vector3(self.wp.pose.pose.position.x - self.state.pose.pose.position.x,
                                          self.wp.pose.pose.position.y - self.state.pose.pose.position.y,
                                          self.wp.pose.pose.position.z - self.state.pose.pose.position.z)

            v_goal.normalize()
            v_goal *= initial_speed

            #####################################################################################################
            # Obstacles
            v_obs_tot = Vector3f.Vector3(0, 0, 0)
            for x in obstacle:
                v_obs = Vector3f.Vector3(x[0] - self.state.pose.pose.position.x,
                                         x[1] - self.state.pose.pose.position.y,
                                         x[2] - self.state.pose.pose.position.z)

                dist = v_obs.norm()
                if dist == 0:
                    dist = 0.00001

                if dist < obs_range:
                    v_obs.normalize();
                    v_obs *= (-1 * (obs_rep * 1 * 1) / (dist ** 2));
                else:
                    v_obs.zero()

                v_obs_tot += v_obs

            #####################################################################################################
            # LJP

            minimum = num_agents
            min_dist = neighbor_range
            v_visc = Vector3f.Vector3(0, 0, 0)
            neighbors = []

            if self.data.data:
                for x in range(0, len(self.data.data), datasize):
                    if self.data.data[x] != idnum:
                        dPos = Vector3f.Vector3(self.data.data[x + 1] - self.state.pose.pose.position.x,
                                                self.data.data[x + 2] - self.state.pose.pose.position.y,
                                                self.data.data[x + 3] - self.state.pose.pose.position.z)

                        dist = dPos.norm()
                        if dist == 0:
                            dist = 0.00001

                        if dist < neighbor_range:
                            neighbors.append(x)

                            if self.data.data[x + 4] < minimum:
                                minimum = self.data.data[x + 4]
                                min_dist = dist

                            if self.data.data[x + 4] == minimum and dist < min_dist:
                                min_dist = dist

            #####################################################################################################
            # Siphon

            queue_value = num_agents
            v_min = Vector3f.Vector3(0, 0, 0)
            static_goal = Vector3f.Vector3(goal_area[0] - self.state.pose.pose.position.x,
                                           goal_area[1] - self.state.pose.pose.position.y,
                                           goal_area[2] - self.state.pose.pose.position.z)

            # if queue_value > 0
            #     queue_value = self.data
            # if static_goal.norm() < goal_range:
            # if idnum == 2:
            #     print "{}     {}     {}".format(queue_value,goalx,self.state.pose.pose.position.x)
            if self.state.pose.pose.position.x < goalx:
                queue_value = 1

            if self.state.pose.pose.position.x < 0 and self.state.pose.pose.position.y > 1.5:
                queue_value = 1

            # if idnum == 2:
            #     print "{}     {}".format(queue_value,self.state.pose.pose.position.x)

            # print "{}     {}".format(queue_value,self.state.pose.pose.position.x)
            if neighbors:
                for x in neighbors:
                    dPos = Vector3f.Vector3(self.data.data[x + 1] - self.state.pose.pose.position.x,
                                            self.data.data[x + 2] - self.state.pose.pose.position.y,
                                            self.data.data[x + 3] - self.state.pose.pose.position.z)

                    dist = dPos.norm()
                    if dist == 0:
                        dist = 0.00001

                    dPos.normalize()
                    dPos *= 24 * etta * (-26 / ((0.6 * (dist + 0.75)) ** 14) + 7 / ((0.6 * (dist + 0.75)) ** 8))

                    # if self.data.data[x+4] < 10000:
                    if self.data.data[x + 4] < queue_value:
                        queue_value = self.data.data[x + 4] + 1

                    if queue_value > num_agents:
                        queue_value = num_agents

                    if self.data.data[x + 4] == minimum and dist == min_dist and minimum < queue_value:
                        v_min = dPos

                    v_visc += dPos

            #####################################################################################################
            # Workspace
            v_bound = Vector3f.Vector3(0, 0, 0)
            if self.state.pose.pose.position.x < workspace[0]:
                v_bound += Vector3f.Vector3(1, 0, 0)
            if self.state.pose.pose.position.x > workspace[1]:
                v_bound += Vector3f.Vector3(-1, 0, 0)
            if self.state.pose.pose.position.y < workspace[2]:
                v_bound += Vector3f.Vector3(0, 1, 0)
            if self.state.pose.pose.position.y > workspace[3]:
                v_bound += Vector3f.Vector3(0, -1, 0)

            #####################################################################################################
            # Full

            v_full = v_goal * weight[0] + v_obs_tot * weight[1] + v_visc * weight[2] + v_min * weight[3] + v_bound * \
                     weight[4]

            if v_full.norm() > max_speed:
                v_full.normalize()
                v_full *= max_speed

            p_full = Vector3f.Vector3(v_full.x + self.state.pose.pose.position.x,
                                      v_full.y + self.state.pose.pose.position.y,
                                      v_full.z + self.state.pose.pose.position.z)

            # print v_full
            #####################################################################################################
            # Publishing
            v = state_18DOF()

            v.header.frame_id = "/world"
            v.pose.pose.position.x = p_full.x
            v.pose.pose.position.y = p_full.y
            v.pose.pose.position.z = height
            v.pose.pose.orientation.w = 1.0
            self.waypoint_publisher.publish(v)

            w = Float32MultiArray()
            w.data = [0] * datasize * num_agents
            if self.data.data:
                for z in range(0, len(w.data), datasize):
                    w.data[z] = z / datasize + 1.0
                    if w.data[z] == idnum:
                        w.data[z + 1] = self.state.pose.pose.position.x
                        w.data[z + 2] = self.state.pose.pose.position.y
                        w.data[z + 3] = self.state.pose.pose.position.z
                        w.data[z + 4] = queue_value
                    else:
                        w.data[z + 1] = self.data.data[z + 1]
                        w.data[z + 2] = self.data.data[z + 2]
                        w.data[z + 3] = self.data.data[z + 3]
                        w.data[z + 4] = self.data.data[z + 4]
                    print
                    "{}     {}".format(w.data[z], w.data[z + 4])
            else:
                w.data = [num_agents] * datasize * num_agents

            print
            "    "
            self.data_publisher.publish(w)
            rate.sleep()

    def state_update(self, data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.state = copy.deepcopy(data)

    def data_update(self, data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.data = copy.deepcopy(data)

    def wp_update(self, data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.wp = copy.deepcopy(data)


if __name__ == '__main__':
    rospy.init_node("waypointstests")
    waypointstest = Waypoints()
    waypointstest.talker()