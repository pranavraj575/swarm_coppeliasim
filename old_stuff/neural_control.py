#!/usr/bin/env python3

import rclpy
import os, sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import time
import threading

from geometry_msgs.msg import Vector3, PoseStamped

# from lta3.msg import state_18DOF
from termcolor import colored
from std_msgs.msg import  Float64#,Float32MultiArray, MultiArrayDimension,
import sys
import copy
import math
from collections import defaultdict

import numpy as np


TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'
TOPIC_DATA = '/shared/data'
TOPIC_ULTRA = '/state/ultrasonic'




SWARM_DATA = defaultdict(lambda: np.array([0.] * 5))


class BlimpNet:
    def __init__(self,
                 agent_id,
                 num_agents,
                 network,
                 swarm_data=SWARM_DATA,
                 debug=False):
        """
        :param agent_id is the number that the agent publishes to
        :param num_agents is the number of agents in the environments
        :param goal_value R^3->R objective function to be minimized
        :param goal_field R^3->R^3, gradient of goal_value, points towards lowest potential (e.g. use radial function if goal is to a point)
        """
        self.agent_id = agent_id
        self.num_agents = num_agents
        self.swarm_data = swarm_data
        self.network=network

        topicGlobal = TOPIC_PRE + str(self.agent_id) + TOPIC_GLOBAL
        topicUltra = TOPIC_PRE + str(self.agent_id) + TOPIC_ULTRA

        topicCmdVel = TOPIC_PRE + str(self.agent_id) + TOPIC_CMD

        self.nodeCtrl = rclpy.create_node('lta' + str(self.agent_id) + '_chain_publisher')  # for publishing

        self.vec_publisher = self.nodeCtrl.create_publisher(Twist, topicCmdVel, 10)
        self.state_keys=('x','y','z','w','ultra')
        self.state = {
            key:0 for key in self.state_keys
        }

        self.nodeState = rclpy.create_node('lta' + str(self.agent_id) + '_chain_reciever')  # for recieving
        self.state_subscriber = self.nodeState.create_subscription(TwistStamped, topicGlobal, self.callbackUpdateState,
                                                                   10)
        self.nodeUltra= rclpy.create_node('lta' + str(self.agent_id) + '_chain_ultra')  # for recieving ultrasound
        self.ultra_subscriber = self.nodeUltra.create_subscription(Float64,
                                                                   topicUltra,
                                                                   self.callbackUpdateUltra,
                                                                   10)
        # for getting state
        self.debug=debug
        if self.debug:
            print('state from', topicGlobal)

        self.spin()

    def break_blimp(self):
        """
        the opposite of the init method
        """
        # destroy them
        self.nodeCtrl.destroy_node()
        self.nodeState.destroy_node()
        self.nodeUltra.destroy_node()


    def callbackUpdateState(self, msg):
        self.state['x'] = msg.twist.linear.x
        self.state['y'] = msg.twist.linear.y
        self.state['z'] = msg.twist.linear.z
        self.state['w'] = msg.twist.angular.z
    def callbackUpdateUltra(self, msg):
        self.state['ultra']=msg.data

    def data_publish(self, agent_data):
        self.swarm_data[self.agent_id] = np.array(agent_data)

    def get_state(self, spin=True):
        if spin:
            self.spin()
        return np.array([self.state[key] for key in self.state_keys])
    def get_state_dim(self):
        return len(self.state_keys)

    def get_position(self, use_ultra=False,spin=True):
        if spin:
            self.spin()

        s = self.state
        return np.array((s['x'], s['y'], s['ultra' if use_ultra else 'z']))

    def move(self, vec):
        msgTwist = Twist()
        msgTwist.linear.x = float(vec[0])
        msgTwist.linear.y = float(vec[1])
        msgTwist.linear.z = float(vec[2])

        self.vec_publisher.publish(msgTwist)


    def spin(self):
        rclpy.spin_once(self.nodeState, timeout_sec=.01)
        rclpy.spin_once(self.nodeUltra, timeout_sec=.01)

    def hover(self):
        # for testing
        self.spin()
        self.move((0, 0, .0))

    def safe_norm(self, vec, d=.00001):
        size = np.linalg.norm(vec)
        if size <= 0:
            size = d
        return vec / size
    def act(self):
        input=self.get_state(spin=True)
        pos=self.get_position(use_ultra=False,spin=False)
        output=self.network(input)
        self.move(output)




from zmqRemoteApi import RemoteAPIClient
import random


def checkForCollisionWrapper(agentHandle, sim):
    collisionResult = None
    collidingObjectHandles = None
    result = sim.checkCollision(agentHandle, sim.handle_all)
    if type(result) is tuple:
        collisionResult = result[0]
        collidingObjectHandles = result[1]
    else:
        collisionResult = result
    collisionResult = bool(collisionResult)
    return collisionResult, collidingObjectHandles


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    startZone = {'xmin': 1, 'xmax': 5,
                 'ymin': -1.5, 'ymax': 4,
                 'zmin': 1, 'zmax': 1,
                 }

    DIR = os.path.join(os.getcwd(), os.path.dirname(sys.argv[0]))
    MODELDIR = DIR + '/ros_ctrl_models/blimpNarrowSensor.ttm'
    SCENEDIR = DIR + '/empty.ttt'
    narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
    modelToLoad = narrowModelPath

    sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
    sim.stopSimulation()
    time.sleep(1)

    sim.loadScene(sceneNamePath)
    time.sleep(1)

    agentHandles = []
    chains = []
    rclpy.init()
    num_agents = 2


    for i in range(num_agents):
        agentHandle = sim.loadModel(modelToLoad)
        for _ in range(100):  # give up at some point
            agentHandles.append(agentHandle)  # Save loaded models' handles for analysis later
            xPos = round(random.uniform(startZone["xmin"], startZone["xmax"]), 2)
            yPos = round(random.uniform(startZone["ymin"], startZone["ymax"]), 2)
            zPos = round(random.uniform(startZone["zmin"], startZone["zmax"]), 2)
            sim.setObjectPosition(agentHandle, -1, [xPos, yPos, zPos])
            collisionResult, collidingObjectHandles = checkForCollisionWrapper(agentHandle, sim)
            if not collisionResult:
                break
        net=lambda pos: np.array((1,2,1))
        pp = BlimpNet(i,
                    num_agents,
                    net
                    )
        chains.append(pp)

    sim.startSimulation()
    time.sleep(1)

    while rclpy.ok():
        s = time.time()
        for i in range(len(chains)):
            chains[i].act()

