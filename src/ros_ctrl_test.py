#!/usr/bin/env python3
import rclpy
import sys
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import time
import numpy as np

PI = 3.14159

TOPIC_PRE = '/swarm/a'
TOPIC_CMD = '/set/cmd_vel'
TOPIC_GLOBAL = '/state/global'

state = {
    'x': 0,
    'y': 0,
    'z': 0,
    'w': 0
}

RECCY = True
POS = []


def callbackUpdateState(msg):
    global state
    global POS
    # print('here')
    state['x'] = msg.twist.linear.x
    state['y'] = msg.twist.linear.y
    state['z'] = msg.twist.linear.z
    state['w'] = msg.twist.angular.z
    if RECCY:
        POS.append(np.array([state['x'], state['y'], state['z'], state['w']]))
    # print(state)


from zmqRemoteApi import RemoteAPIClient
import os


def main(args=None):
    global state

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    RESET = True

    DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
    MODELDIR = os.path.join(DIR, 'ros_ctrl_models', 'blimp_narrow.ttm')
    SCENEDIR = os.path.join(DIR, 'scenes', 'empty.ttt')

    narrowModelPath = os.path.abspath(os.path.expanduser(MODELDIR))
    modelToLoad = narrowModelPath

    sceneNamePath = os.path.abspath(os.path.expanduser(SCENEDIR))
    sim.stopSimulation()
    time.sleep(1)
    if RESET:
        sim.loadScene(sceneNamePath)
        time.sleep(1)

        agentHandle = sim.loadModel(modelToLoad)
    agent = '0'

    topicCmdVel = TOPIC_PRE + agent + TOPIC_CMD
    topicGlobal = TOPIC_PRE + agent + TOPIC_GLOBAL

    rclpy.init(args=args)

    NODE = rclpy.create_node('test')
    publisherAlign = NODE.create_publisher(Twist, topicCmdVel, 10)
    subscriberPos = NODE.create_subscription(TwistStamped, topicGlobal, callbackUpdateState, 10)

    DT = 50 / 1000
    sim.startSimulation()
    test = np.array([.0, .0, .0, np.pi])
    f = 1.
    for _ in range(500):
        rclpy.spin_once(NODE, timeout_sec=0.01)

        msgTwist = Twist()

        msgTwist.linear.x = test[0] * f
        msgTwist.linear.y = test[1] * f
        msgTwist.linear.z = test[2] * f
        msgTwist.angular.z = test[3]
        msgTwist.angular.x = 0.  # NOTE: this tells the blimp that we care about heading
        publisherAlign.publish(msgTwist)
        time.sleep(.01)

    sim.stopSimulation()

    from matplotlib import pyplot as plt
    leg = []
    # plt.plot([r[k] for r in RECORD]); leg.append('POSITION')
    VEL = []
    for i in range(len(POS) - 1):
        VEL.append((POS[i + 1] - POS[i]) / DT)
    # plt.plot([v[k] for v in VEL]); leg.append('VELOCITY')
    ACC = []
    for i in range(len(VEL) - 1):
        ACC.append((VEL[i + 1] - VEL[i]) / DT)
    # ACC=ACC[-50:]
    CONTROL = VEL
    fun = lambda v: v[-1]
    fun = None

    if fun is None:
        for d in range(4):
            plt.plot([v[d] for v in CONTROL])
            leg.append(['x', 'y', 'z', 'w'][d])
    else:
        plt.plot([fun(v) for v in CONTROL])
        leg.append('CONTROL')
    if fun is not None:
        print(sum([fun(v) for v in CONTROL]) / len(CONTROL))
        print('goal', fun(test))
    plt.legend(leg)
    plt.show()


if __name__ == '__main__':
    main()
