#!/usr/bin/env python3
import rclpy
import sys, subprocess, psutil
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


def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


def param1(num):
    return '-gREMOTEAPISERVERSERVICE_' + str(num) + '_FALSE_TRUE'


def param2(num=23050, step=0):
    return ' -GwsRemoteApi.port=' + str(num + step)


def param(num=23000, step=0):
    return ' -GzmqRemoteApi.rpcPort=' + str(num + step)


cmd = '/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh'
STEP = 2
p = subprocess.Popen(cmd + param() + param2(), stdout=subprocess.PIPE, shell=True)
q = subprocess.Popen(cmd + param(step=STEP) + param2(step=STEP), stdout=subprocess.PIPE, shell=True)
from src.swarm_expiriment import *
import threading

bb = blimpTest(10, lambda i: ((-5, 5), (-5, 5), (1, 5)), command=(0, 0, .1), simId=23000)
bb2 = blimpTest(10, lambda i: ((-5, 5), (-5, 5), (1, 5)), command=(0, 0, -.1), simId=23000 + STEP)
tt = threading.Thread(target=lambda: bb.run_exp(end_time=lambda t: False))
tt2 = threading.Thread(target=lambda: bb2.run_exp(end_time=lambda t: False))
tt.start()
time.sleep(1)  # vaguely important?
tt2.start()
while True:
    time.sleep(1)

client = RemoteAPIClient(port=23000)
sim = client.getObject('sim')
from zmqRemoteApi import RemoteAPIClient

client2 = RemoteAPIClient(port=23000 + STEP)
sim2 = client2.getObject('sim')

TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
MODELDIR = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')
SCENEDIR = os.path.join(DIR, 'scenes', 'empty.ttt')
SCENEDIR2 = os.path.join(DIR, 'scenes', 'poles.ttt')

narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
modelToLoad = narrowModelPath

sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
sceneNamePath2 = os.path.abspath(os.path.expanduser(SCENEDIR2))

time.sleep(1)
sim.loadScene(sceneNamePath)
sim2.loadScene(sceneNamePath2)  # +'@keepCurrent')
time.sleep(1)

agentHandle = sim.loadModel(modelToLoad)
agentHandle2 = sim2.loadModel(modelToLoad)
# TODO: NOT fine that the topics are the same
id1 = '23000'
agent = '0'

topicCmdVel = TOPIC_PRE + id1 + '_' + agent + TOPIC_CMD
topicGlobal = TOPIC_PRE + id1 + '_' + agent + TOPIC_GLOBAL

rclpy.init()

NODE = rclpy.create_node('test')
publisherAlign = NODE.create_publisher(Twist, topicCmdVel, 10)
subscriberPos = NODE.create_subscription(TwistStamped, topicGlobal, callbackUpdateState, 10)

DT = 50/1000
sim.startSimulation()
sim2.startSimulation()
for _ in range(5000):
    rclpy.spin_once(NODE, timeout_sec=0.01)

    msgTwist = Twist()

    msgTwist.linear.x = .1
    msgTwist.linear.y = 0.
    msgTwist.linear.z = 0.
    publisherAlign.publish(msgTwist)
    time.sleep(.01)

kill(p.pid)
kill(q.pid)
