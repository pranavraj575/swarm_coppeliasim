#!/usr/bin/env python3
import rclpy
import sys, subprocess,psutil
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
    return '-gREMOTEAPISERVERSERVICE_'+str(num)+'_FALSE_TRUE'
def param(num):
    return '-GzmqRemoteApi.rpcPort='+str(num)
port1=19997
port2=19998
cmd='/home/rajbhandari/Downloads/CoppeliaSim_Edu_V4_3_0_rev12_Ubuntu20_04/coppeliaSim.sh '
p = subprocess.Popen(cmd+param(port1), stdout=subprocess.PIPE, shell=True)
input()
q= subprocess.Popen(cmd+param(port2), stdout=subprocess.PIPE, shell=True)

input()
print('getting things')
client = RemoteAPIClient(port=port1)
sim = client.getObject('sim')

client2 = RemoteAPIClient(port=port2)
sim2 = client.getObject('sim')
print('done')
input()
sim2.startSimulation()
sim.startSimulation()
print('starting both')
input()
sim2.stopSimulation()
sim.pauseSimulation()
print('stopping/pausing')
input()
kill(p.pid)
kill(q.pid)
quit()
TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'


DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
MODELDIR = os.path.join(DIR, 'ros_ctrl_models', 'blimpNarrowSensor.ttm')
SCENEDIR = os.path.join(DIR, 'scenes', 'empty.ttt')
SCENEDIR2 = os.path.join(DIR, 'scenes', 'poles.ttt')

narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
modelToLoad = narrowModelPath

sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
sceneNamePath2 = os.path.abspath(os.path.expanduser(SCENEDIR2))
sim.stopSimulation()

time.sleep(1)
# sim.loadScene(sceneNamePath)
sim2.loadScene(sceneNamePath2)  # +'@keepCurrent')
time.sleep(1)

agentHandle = sim.loadModel(modelToLoad)
agent = '0'

topicCmdVel = TOPIC_PRE + agent + TOPIC_CMD
topicGlobal = TOPIC_PRE + agent + TOPIC_GLOBAL

rclpy.init()

NODE = rclpy.create_node('test')
publisherAlign = NODE.create_publisher(Twist, topicCmdVel, 10)
subscriberPos = NODE.create_subscription(TwistStamped, topicGlobal, callbackUpdateState, 10)

DT = 50 / 1000
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
