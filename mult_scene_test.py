#!/usr/bin/env python3
import rclpy
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import time
import numpy as np
from zmqRemoteApi import RemoteAPIClient
import os

state = {
    'x': 0,
    'y': 0,
    'z': 0,
    'w': 0
}

def callbackUpdateState(msg):
    global state
    state['x'] = msg.twist.linear.x
    state['y'] = msg.twist.linear.y
    state['z'] = msg.twist.linear.z
    state['w'] = msg.twist.angular.z

TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'


client = RemoteAPIClient()
sim = client.getObject('sim')
client2 = RemoteAPIClient(port=23005)
sim2 = client.getObject('sim')

DIR = os.path.join(os.getcwd(), os.path.dirname(sys.argv[0]))
MODELDIR = DIR + '/ros_ctrl_models/blimpNarrowSensor.ttm'
SCENEDIR = DIR + '/empty.ttt'
SCENEDIR2=DIR+'/poles.ttt'

narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
modelToLoad = narrowModelPath

sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
sceneNamePath2 = os.path.abspath(os.path.expanduser(SCENEDIR2))
sim.stopSimulation()

time.sleep(1)
#sim.loadScene(sceneNamePath)
sim2.loadScene(sceneNamePath2)#+'@keepCurrent')
time.sleep(1)

agentHandle = sim.loadModel(modelToLoad)
agent='0'

topicCmdVel = TOPIC_PRE + agent + TOPIC_CMD
topicGlobal = TOPIC_PRE + agent + TOPIC_GLOBAL

rclpy.init()

NODE=rclpy.create_node('test')
publisherAlign = NODE.create_publisher(Twist, topicCmdVel, 10)
subscriberPos = NODE.create_subscription(TwistStamped, topicGlobal, callbackUpdateState, 10)

DT=50/1000
sim.startSimulation()
for _ in range(500):
    rclpy.spin_once(NODE, timeout_sec=0.01)

    msgTwist = Twist()

    msgTwist.linear.x = 1.
    msgTwist.linear.y = 0.
    msgTwist.linear.z = 0.
    publisherAlign.publish(msgTwist)
    time.sleep(.01)

sim.stopSimulation()
